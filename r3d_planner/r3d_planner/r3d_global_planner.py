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
from ament_index_python.packages import get_package_share_directory

class GlobalGraphPlanner(Node):
    def __init__(self):
        super().__init__('global_graph_planner')
        
        # --- PFAD KONFIGURATION (DYNAMISCH) ---
        
        # 1. Wir suchen dynamisch den Pfad zum "share"-Ordner des Preprocessor-Pakets
        # Das funktioniert egal wo der Workspace liegt!
        try:
            preprocessor_share = get_package_share_directory('r3d_preprocessor')
            default_dir = os.path.join(preprocessor_share, 'maps')
        except Exception as e:
            # Fallback, falls das Package nicht gefunden wird (z.B. nicht gesourced)
            self.get_logger().error(f"Konnte r3d_preprocessor nicht finden: {e}")
            default_dir = "/tmp" # Oder ein anderer Fallback

        # 2. Parameter wie gehabt
        self.declare_parameter('map_dir', default_dir)
        self.declare_parameter('map_name', 'graph.pkl')
        
        # 3. Parameter abrufen
        map_dir = self.get_parameter('map_dir').get_parameter_value().string_value
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        
        # 4. Pfad zusammenbauen
        full_path = os.path.join(map_dir, map_name)
        
        self.get_logger().info(f"Versuche Karte zu laden von: {full_path}")
        self.load_graph(full_path)
        
        # --- VISUALISIERUNG ---
        self.pub_viz_path = self.create_publisher(Marker, '/planned_path', 10)
        self.path_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        # --- ACTION SERVER ---
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback
        )
        self.get_logger().info('Global 3D Graph Planner ready.')

    # ... (Rest der Klasse: load_graph, a_star etc. bleiben exakt gleich) ...
    
    def load_graph(self, path):
        try:
            if not os.path.exists(path):
                self.get_logger().error(f'DATEI NICHT GEFUNDEN: {path}')
                return

            with open(path, 'rb') as f:
                self.graph = pickle.load(f)
            
            self.node_ids = list(self.graph.keys())
            self.coords = [self.graph[nid]['pos'] for nid in self.node_ids]
            self.kdtree = KDTree(self.coords)
            self.get_logger().info(f'Graph erfolgreich geladen ({len(self.node_ids)} Knoten).')
        except Exception as e:
            self.get_logger().error(f'Fehler beim Laden des Graphen: {e}')

    def get_nearest_node(self, point):
        # Safety check falls Graph nicht geladen
        if not hasattr(self, 'kdtree'): return None
        dist, idx = self.kdtree.query([point.x, point.y, point.z])
        return self.node_ids[idx]

    def heuristic(self, a_id, b_id):
        pos_a = np.array(self.graph[a_id]['pos'])
        pos_b = np.array(self.graph[b_id]['pos'])
        return np.linalg.norm(pos_a - pos_b)

    def a_star(self, start_id, goal_id):
        if start_id is None or goal_id is None: return None
        
        open_set = []
        heapq.heappush(open_set, (0, start_id))
        came_from = {}
        g_score = {node: float('inf') for node in self.graph}
        g_score[start_id] = 0
        
        while open_set:
            current_score, current = heapq.heappop(open_set)
            
            if current == goal_id:
                return self.reconstruct_path(came_from, current)
            
            for neighbor, cost in self.graph[current]['neighbors']:
                tentative_g_score = g_score[current] + cost
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, goal_id)
                    heapq.heappush(open_set, (f_score, neighbor))
        return None

    def reconstruct_path(self, came_from, current):
        path_ids = [current]
        while current in came_from:
            current = came_from[current]
            path_ids.append(current)
        path_ids.reverse()
        return path_ids

    def execute_callback(self, goal_handle):
        self.get_logger().info('Berechne 3D Pfad...')
        goal_req = goal_handle.request
        start_pose = goal_req.start.pose
        goal_pose = goal_req.goal.pose
        
        start_node = self.get_nearest_node(start_pose.position)
        end_node = self.get_nearest_node(goal_pose.position)
        
        if not start_node or not end_node:
             self.get_logger().error("Graph nicht bereit oder Start/Ziel ungültig.")
             goal_handle.abort()
             return ComputePathToPose.Result()

        path_ids = self.a_star(start_node, end_node)
        
        result = ComputePathToPose.Result()
        
        if not path_ids:
            self.get_logger().warn('A* hat keinen Pfad gefunden!')
            goal_handle.abort()
            return result

        # --- Visualisierung ---
        self.publish_visual_path(path_ids, goal_req.header.frame_id)
        
        # Nav2 Path Message
        path_msg = Path()
        path_msg.header.frame_id = goal_req.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for nid in path_ids:
            x, y, z = self.graph[nid]['pos']
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)
            
        result.path = path_msg
        goal_handle.succeed()
        return result

    def publish_visual_path(self, path_ids, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "graph_path"
        marker.id = 0
        marker.type = Marker.CUBE_LIST # Zeigt Voxel an
        marker.action = Marker.ADD
        marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1
        marker.color = self.path_color
        marker.lifetime.sec = 0

        for nid in path_ids:
            x, y, z = self.graph[nid]['pos']
            p = Point()
            p.x = float(x); p.y = float(y); p.z = float(z)
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
