import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import pickle
import numpy as np
import heapq
from scipy.spatial import KDTree
import os
import networkx as nx
from ament_index_python.packages import get_package_share_directory

class GlobalGraphPlanner(Node):
    def __init__(self):
        super().__init__('global_graph_planner')
        
        # --- PFAD KONFIGURATION ---
        try:
            preprocessor_share = get_package_share_directory('r3d_preprocessor')
            default_dir = os.path.join(preprocessor_share, 'maps')
        except Exception as e:
            self.get_logger().error(f"Konnte r3d_preprocessor nicht finden: {e}")
            default_dir = "/tmp"

        self.declare_parameter('map_dir', default_dir)
        self.declare_parameter('map_name', 'nav_graph.pkl') 
        
        map_dir = self.get_parameter('map_dir').get_parameter_value().string_value
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        
        if os.path.isabs(os.path.expanduser(map_name)):
            full_path = os.path.expanduser(map_name)
        else:
            full_path = os.path.join(map_dir, map_name)
        
        self.load_graph(full_path)
        
        # --- PUBLISHER ---
        # 1. Für RViz (die Würfel)
        self.pub_viz_path = self.create_publisher(Marker, '/planned_path', 10)
        self.path_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        # 2. Für den Path Follower (die mathematische Route)
        self.pub_path = self.create_publisher(Path, '/global_path', 10)

        # --- ACTION SERVER ---
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback
        )
        self.get_logger().info('Global 3D Graph Planner ready.')

    def load_graph(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'DATEI NICHT GEFUNDEN: {path}')
            return

        try:
            with open(path, 'rb') as f:
                data = pickle.load(f)
            
            if isinstance(data, dict) and "graph" in data:
                self.graph = data["graph"]      # NetworkX Graph
                self.origin = data["origin"]    # np.array([x,y,z])
                self.voxel_size = data["voxel_size"] # float (z.B. 0.05)
            else:
                self.get_logger().error("Falsches Dateiformat! Erwarte Dictionary mit 'graph', 'origin', 'voxel_size'.")
                return

            self.node_list = list(self.graph.nodes())
            self.world_coords = []
            
            for (vx, vy, vz) in self.node_list:
                x = self.origin[0] + vx * self.voxel_size + self.voxel_size/2
                y = self.origin[1] + vy * self.voxel_size + self.voxel_size/2
                z = self.origin[2] + vz * self.voxel_size + self.voxel_size/2
                self.world_coords.append([x, y, z])
            
            self.kdtree = KDTree(self.world_coords)
            self.get_logger().info(f'Graph geladen: {len(self.node_list)} Knoten. VoxelSize: {self.voxel_size}m')
            
        except Exception as e:
            self.get_logger().error(f'CRITICAL ERROR beim Laden: {e}')

    def get_nearest_node(self, point):
        if not hasattr(self, 'kdtree'): return None
        dist, idx = self.kdtree.query([point.x, point.y, point.z])
        return self.node_list[idx]

    def heuristic(self, node_a, node_b):
        vec_a = np.array(node_a)
        vec_b = np.array(node_b)
        return np.linalg.norm(vec_a - vec_b) * self.voxel_size

    def a_star(self, start_node, goal_node):
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {node: float('inf') for node in self.graph.nodes()}
        g_score[start_node] = 0
        
        while open_set:
            current_score, current = heapq.heappop(open_set)
            
            if current == goal_node:
                return self.reconstruct_path(came_from, current)
            
            for neighbor, attributes in self.graph[current].items():
                cost = attributes.get('weight', 1.0)
                
                tentative_g_score = g_score[current] + cost
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, goal_node)
                    heapq.heappush(open_set, (f_score, neighbor))
        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def execute_callback(self, goal_handle):
        self.get_logger().info('Berechne Pfad...')
        goal_req = goal_handle.request
        
        if not hasattr(self, 'graph'):
            goal_handle.abort()
            return ComputePathToPose.Result()

        start_node = self.get_nearest_node(goal_req.start.pose.position)
        end_node = self.get_nearest_node(goal_req.goal.pose.position)
        
        path_ids = self.a_star(start_node, end_node)
        
        result = ComputePathToPose.Result()
        if not path_ids:
            self.get_logger().warn('Kein Pfad gefunden!')
            goal_handle.abort()
            return result

        self.publish_visual_path(path_ids, goal_req.goal.header.frame_id)
        
        path_msg = Path()
        path_msg.header.frame_id = goal_req.goal.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for (vx, vy, vz) in path_ids:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = self.origin[0] + vx * self.voxel_size + self.voxel_size/2
            pose.pose.position.y = self.origin[1] + vy * self.voxel_size + self.voxel_size/2
            pose.pose.position.z = self.origin[2] + vz * self.voxel_size + self.voxel_size/2
            
            # --- NEU: Engstellen-Flag in der Orientierung verstecken ---
            node_type = self.graph.nodes[(vx, vy, vz)].get('type', 'floor')
            if node_type == 'narrow':
                pose.pose.orientation.z = 1.0  # 1.0 bedeutet "Achtung: Engstelle!"
            else:
                pose.pose.orientation.z = 0.0  # 0.0 bedeutet "Freie Fahrt"
                
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)
        
        result.path = path_msg
        goal_handle.succeed()
        return result

    def publish_visual_path(self, path_ids, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "graph_path"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color = self.path_color
        
        for (vx, vy, vz) in path_ids:
            p = Point()
            p.x = self.origin[0] + vx * self.voxel_size + self.voxel_size/2
            p.y = self.origin[1] + vy * self.voxel_size + self.voxel_size/2
            p.z = self.origin[2] + vz * self.voxel_size + self.voxel_size*2
            marker.points.append(p)

        self.pub_viz_path.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalGraphPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
