import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import networkx as nx
import pickle
import numpy as np
import os

class VoxelMapPublisher(Node):
    def __init__(self):
        super().__init__('voxel_map_publisher')
        
        # Parameter deklarieren
        self.declare_parameter('graph_path', 'nav_graph.pkl')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(Marker, '/global_voxel_map', qos_profile)
        self.edge_pub = self.create_publisher(Marker, '/global_graph_edges', qos_profile)
        
        self.G = None
        self.origin = np.array([0.0, 0.0, 0.0])
        self.voxel_size = 0.05
        
        self.load_and_publish()

    def load_and_publish(self):
        # Parameter holen
        pkl_file = self.get_parameter('graph_path').get_parameter_value().string_value
        pkl_file = os.path.abspath(os.path.expanduser(pkl_file))

        self.get_logger().info(f'Lade Datei: {pkl_file}...')
        
        if not os.path.exists(pkl_file):
            self.get_logger().error(f"Datei nicht gefunden: {pkl_file}")
            return

        try:
            with open(pkl_file, 'rb') as f:
                data = pickle.load(f)
                
            if isinstance(data, dict):
                self.G = data["graph"]
                self.origin = data["origin"]
                self.voxel_size = data["voxel_size"]
                self.get_logger().info(f"Metadaten geladen! Origin: {self.origin}, VoxelSize: {self.voxel_size}")
            else:
                self.G = data
                self.get_logger().warn("Altes Dateiformat! Origin ist unbekannt.")
                
        except Exception as e:
            self.get_logger().error(f'Fehler beim Laden: {e}')
            return

        self.get_logger().info(f'Graph hat {self.G.number_of_nodes()} Knoten. Generiere Marker...')

        # --- 1. VOXELS ---
        voxel_marker = Marker()
        voxel_marker.header.frame_id = "map"
        voxel_marker.header.stamp = self.get_clock().now().to_msg()
        voxel_marker.type = Marker.CUBE_LIST
        voxel_marker.action = Marker.ADD
        voxel_marker.scale.x = self.voxel_size * 0.9
        voxel_marker.scale.y = self.voxel_size * 0.9
        voxel_marker.scale.z = self.voxel_size * 0.9
        
        color_floor = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)
        color_stair = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
        
        for node, data in self.G.nodes(data=True):
            x = self.origin[0] + node[0] * self.voxel_size + self.voxel_size/2
            y = self.origin[1] + node[1] * self.voxel_size + self.voxel_size/2
            z = self.origin[2] + node[2] * self.voxel_size + self.voxel_size/2
            
            p = Point(x=x, y=y, z=z)
            voxel_marker.points.append(p)
            
            if data.get('type') == 'stair_access':
                voxel_marker.colors.append(color_stair)
            else:
                voxel_marker.colors.append(color_floor)

        # --- 2. EDGES ---
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.type = Marker.LINE_LIST
        edge_marker.scale.x = 0.02
        edge_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        for u, v, data in self.G.edges(data=True):
            if data.get('type') == 'stair':
                u_pos = self.origin + np.array(u) * self.voxel_size + self.voxel_size/2
                v_pos = self.origin + np.array(v) * self.voxel_size + self.voxel_size/2
                edge_marker.points.append(Point(x=u_pos[0], y=u_pos[1], z=u_pos[2]))
                edge_marker.points.append(Point(x=v_pos[0], y=v_pos[1], z=v_pos[2]))

        self.marker_pub.publish(voxel_marker)
        self.edge_pub.publish(edge_marker)
        self.get_logger().info('Marker veröffentlicht!')

def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
