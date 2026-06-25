import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import pickle
import os

class VoxelMapPublisher(Node):
    def __init__(self):
        super().__init__('voxel_map_publisher')
        self.declare_parameter('graph_path', '')
        
        # QoS Profil (Transient Local), damit RViz die Marker sofort nach dem Verbinden lädt
        qos_profile = QoSProfile(
            depth=1, 
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Wieder der normale Marker!
        self.pub = self.create_publisher(Marker, '/r3d_global_voxel_map', qos_profile)
        
        self.timer = self.create_timer(2.0, self.publish_markers)
        
        self.marker = None
        self.load_graph_data()

    def load_graph_data(self):
        graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        if not os.path.exists(graph_path):
            self.get_logger().error(f"Datei nicht gefunden: {graph_path}")
            return

        self.get_logger().info(f"Lade Datei: {graph_path}...")
        try:
            with open(graph_path, 'rb') as f:
                data = pickle.load(f)
            
            self.G = data['graph']
            self.origin = data['origin']
            self.voxel_size = data['voxel_size']
            
            # Falls du die statischen Hindernisse aus dem Preprocessor in RViz doch nicht 
            # sehen möchtest, kannst du die Schleife unten für die Hindernisse einfach löschen.
            self.obstacles = data.get('obstacles', set())
            
            self.get_logger().info(f"Metadaten geladen! Origin: {self.origin}, VoxelSize: {self.voxel_size}")
            self.get_logger().info(f"Graph hat {len(self.G.nodes)} Knoten.")
            
            self.construct_marker()
            
        except Exception as e:
            self.get_logger().error(f"Fehler beim Laden: {e}")

    def construct_marker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "voxel_map"
        self.marker.id = 0
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = self.voxel_size
        self.marker.scale.y = self.voxel_size
        self.marker.scale.z = self.voxel_size
        
        self.marker.color.a = 1.0

        # Farben (100% Deckkraft)
        c_floor  = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) # Grün
        c_narrow = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) # Hellblau (Cyan)
        c_stair  = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0) # Gelb
        c_obst   = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0) # Pink
        
        # 1. Begehbare Bereiche & Graphen-Topologie
        for node, n_data in self.G.nodes(data=True):
            x = self.origin[0] + node[0] * self.voxel_size + self.voxel_size/2
            y = self.origin[1] + node[1] * self.voxel_size + self.voxel_size/2
            z = self.origin[2] + node[2] * self.voxel_size + self.voxel_size/2
            self.marker.points.append(Point(x=x, y=y, z=z))
            
            # Jedem Punkt seine individuelle Farbe zuweisen
            node_type = n_data.get('type')
            if node_type == 'stair_access':
                self.marker.colors.append(c_stair)
            elif node_type == 'narrow':
                self.marker.colors.append(c_narrow)
            else:
                self.marker.colors.append(c_floor)

        # 2. Statische Hindernisse (Pink)
        # (Wenn dir der r3d_planner Local Filter für Hindernisse reicht, 
        # kannst du diesen Block auskommentieren, dann ist die Karte noch "leichter")
        for (vx, vy, vz) in self.obstacles:
            x = self.origin[0] + vx * self.voxel_size + self.voxel_size/2
            y = self.origin[1] + vy * self.voxel_size + self.voxel_size/2
            z = self.origin[2] + vz * self.voxel_size + self.voxel_size/2
            self.marker.points.append(Point(x=x, y=y, z=z))
            self.marker.colors.append(c_obst)

    def publish_markers(self):
        if self.marker is not None:
            self.pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
