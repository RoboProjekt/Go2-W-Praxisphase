import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import open3d as o3d
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
        self.declare_parameter('map_name', 'map_analysed.pcd') 
        
        # Da die PCD keine Metadaten enthält, müssen wir dem Planner die 
        # Rastergrößen mitteilen, damit er den Graph korrekt rekonstruieren kann.
        self.declare_parameter('voxel_size_cm', 5.0)
        self.declare_parameter('min_step_height_cm', 5.0)
        self.declare_parameter('max_step_height_cm', 25.0)
        
        self.voxel_size = self.get_parameter('voxel_size_cm').get_parameter_value().double_value / 100.0
        self.min_stair_height = self.get_parameter('min_step_height_cm').get_parameter_value().double_value / 100.0
        self.max_stair_height = self.get_parameter('max_step_height_cm').get_parameter_value().double_value / 100.0

        map_dir = self.get_parameter('map_dir').get_parameter_value().string_value
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        
        if os.path.isabs(os.path.expanduser(map_name)):
            full_path = os.path.expanduser(map_name)
        else:
            full_path = os.path.join(map_dir, map_name)
        
        self.load_and_rebuild_graph_from_pcd(full_path)
        
        # --- PUBLISHER ---
        self.pub_viz_path = self.create_publisher(Marker, '/planned_path', 10)
        self.path_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        self.pub_path = self.create_publisher(Path, '/global_path', 10)

        # --- ACTION SERVER ---
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback
        )
        self.get_logger().info('Global 3D Graph Planner ready (PCD Mode).')

    def load_and_rebuild_graph_from_pcd(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'DATEI NICHT GEFUNDEN: {path}')
            return

        self.get_logger().info(f"Lade PCD und rekonstruiere Topologie: {path} ... Das kann kurz dauern.")
        try:
            pcd = o3d.io.read_point_cloud(path)
            if pcd.is_empty() or not pcd.has_colors():
                self.get_logger().error("Punktwolke ist leer oder hat keine Farben (RGB)!")
                return

            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            
            self.graph = nx.Graph()
            self.node_list = []
            self.world_coords = []
            xy_map = {}

            # 1. Knoten aus Farben extrahieren
            for i in range(len(points)):
                r, g, b = colors[i]
                
                # Tolerante Farberkennung (Pink = Hindernis -> überspringen)
                if r > 0.8 and g < 0.2 and b > 0.8:
                    continue
                    
                node_type = "floor"
                if r < 0.2 and g > 0.8 and b > 0.8:     # Türkis
                    node_type = "narrow"
                elif r > 0.8 and g > 0.6 and b < 0.2:   # Gelb
                    node_type = "stair_access"

                x, y, z = points[i]
                
                # Erstelle stabilen, grid-basierten Integer-Key für NetworkX
                vx = int(round(x / self.voxel_size))
                vy = int(round(y / self.voxel_size))
                vz = int(round(z / self.voxel_size))
                node_key = (vx, vy, vz)
                
                self.graph.add_node(node_key, type=node_type, pos=(x, y, z))
                self.node_list.append(node_key)
                self.world_coords.append([x, y, z])
                
                if (vx, vy) not in xy_map:
                    xy_map[(vx, vy)] = []
                xy_map[(vx, vy)].append(vz)

            # 2. Kanten (Edges) und Gewichte rekonstruieren
            neighbor_offsets = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for current_node in self.node_list:
                cx, cy, cz = current_node
                node_type = self.graph.nodes[current_node]['type']
                
                for dx, dy in neighbor_offsets:
                    nx_idx, ny_idx = cx + dx, cy + dy
                    if (nx_idx, ny_idx) in xy_map:
                        for nz in xy_map[(nx_idx, ny_idx)]:
                            delta_z_meters = (nz - cz) * self.voxel_size
                            abs_delta_z = abs(delta_z_meters)
                            dist_xy = np.sqrt(dx**2 + dy**2) * self.voxel_size
                            
                            if abs_delta_z <= self.min_stair_height:
                                weight = np.sqrt(dist_xy**2 + delta_z_meters**2)
                                n_type = self.graph.nodes[(nx_idx, ny_idx, nz)]['type']
                                if node_type == "narrow" or n_type == "narrow":
                                    weight *= 1.2
                                self.graph.add_edge(current_node, (nx_idx, ny_idx, nz), weight=weight)
                                
                            elif self.min_stair_height < abs_delta_z <= self.max_stair_height:
                                weight = np.sqrt(dist_xy**2 + delta_z_meters**2) * 3.0
                                self.graph.add_edge(current_node, (nx_idx, ny_idx, nz), weight=weight)

            self.kdtree = KDTree(self.world_coords)
            self.get_logger().info(f'Graph erfolgreich rekonstruiert: {len(self.node_list)} begehbare Knoten gefunden.')
            
        except Exception as e:
            self.get_logger().error(f'CRITICAL ERROR beim Laden der PCD: {e}')

    def get_nearest_node(self, point):
        if not hasattr(self, 'kdtree'): return None
        dist, idx = self.kdtree.query([point.x, point.y, point.z])
        return self.node_list[idx]

    def heuristic(self, node_a, node_b):
        vec_a = np.array(self.graph.nodes[node_a]['pos'])
        vec_b = np.array(self.graph.nodes[node_b]['pos'])
        return np.linalg.norm(vec_a - vec_b)

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
        
        for node_key in path_ids:
            pose = PoseStamped()
            pose.header = path_msg.header
            
            # Koordinaten direkt aus dem Graph holen
            pos = self.graph.nodes[node_key]['pos']
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            
            node_type = self.graph.nodes[node_key].get('type', 'floor')
            if node_type == 'narrow':
                pose.pose.orientation.z = 1.0  
            else:
                pose.pose.orientation.z = 0.0  
                
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
        # Ändere den Typ von Würfeln zu einer zusammenhängenden Linie
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Bei einer Linie bestimmt scale.x die Liniendicke
        marker.scale.x = 0.05 
        
        # Farbe: Dunkelblau (RGB 0,0,128) und voll sichtbar (a=1.0)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=0.5, a=1.0)
        
        for node_key in path_ids:
            p = Point()
            pos = self.graph.nodes[node_key]['pos']
            p.x = pos[0]
            p.y = pos[1]
            # Visueller Offset, damit der Pfad leicht über dem Boden schwebt
            p.z = pos[2] + (self.voxel_size * 1.5) 
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
