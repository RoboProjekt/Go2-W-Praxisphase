import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, DurabilityPolicy
import open3d as o3d
import numpy as np
import os
import struct

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        
        # Parameter für den Dateipfad
        self.declare_parameter('pcd_path', 'environment.pcd')
        
        # QoS: TRANSIENT_LOCAL ist wichtig, damit RViz die Karte auch sieht, 
        # wenn es erst nach dem Start geöffnet wird (Latched Topic).
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(PointCloud2, '/map_pointcloud', qos_profile)
        
        # Laden und Publishen
        self.load_and_publish()

    def load_and_publish(self):
        pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        pcd_path = os.path.abspath(os.path.expanduser(pcd_path))
        
        self.get_logger().info(f"Lade PCD Datei: {pcd_path}")
        
        if not os.path.exists(pcd_path):
            self.get_logger().error("Datei nicht gefunden!")
            return

        try:
            # 1. Mit Open3D laden
            pcd = o3d.io.read_point_cloud(pcd_path)
            if pcd.is_empty():
                self.get_logger().error("Punktwolke ist leer.")
                return

            # 2. In NumPy konvertieren
            points = np.asarray(pcd.points)
            self.get_logger().info(f"PCD geladen mit {len(points)} Punkten.")

            # Optional: Farben laden (falls vorhanden)
            colors = []
            if pcd.has_colors():
                colors = np.asarray(pcd.colors) # Open3D speichert RGB als 0.0-1.0 float

            # 3. PointCloud2 Nachricht erstellen
            msg = self.create_pc2_msg(points, colors)
            
            # 4. Publishen
            self.pub.publish(msg)
            self.get_logger().info("PCD erfolgreich auf /map_pointcloud veröffentlicht.")

        except Exception as e:
            self.get_logger().error(f"Fehler beim Laden/Publishen: {e}")

    def create_pc2_msg(self, points, colors):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" # Wichtig: Muss im Map-Frame liegen

        # Layout definieren
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Daten vorbereiten
        if len(colors) > 0:
            # Wenn Farbe dabei ist, müssen wir RGB packen (etwas komplexer in Python pur)
            # Der Einfachheit halber publishen wir hier erst mal nur XYZ,
            # um Struct-Packing Fehler zu vermeiden. Open3D Farben sind float, ROS braucht gepackte Ints.
            # Für Visualisierung reicht XYZ + "Flat Color" in RViz oft aus.
            pass 

        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12 # 3 * float32 (4 bytes)
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        
        # Konvertieren zu Bytes (float32)
        msg.data = points.astype(np.float32).tobytes()
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
