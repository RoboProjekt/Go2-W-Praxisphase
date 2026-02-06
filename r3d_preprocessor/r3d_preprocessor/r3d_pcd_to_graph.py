import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
import networkx as nx
import pickle
import sys
import os

# --- Deine Logik-Klasse (bleibt fast unverändert) ---
class VoxelNavigationGraph:
    def __init__(self, pcd_path, voxel_size=0.05, max_stair_height=0.25, logger=None):
        self.pcd_path = pcd_path
        self.voxel_size = voxel_size
        self.min_stair_height = 0.05 
        self.max_stair_height = max_stair_height
        self.robot_height_clearance = 1.0
        
        self.G = nx.Graph()
        self.voxel_grid = None
        self.occupied_indices = set()
        self.walkable_nodes = set()
        self.origin = np.array([0.0, 0.0, 0.0])
        
        # Logging Helper, damit wir ROS Logger nutzen können
        self.logger = logger

    def log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def load_and_voxelize(self):
        self.log(f"Lade Punktwolke: {self.pcd_path}")
        try:
            pcd = o3d.io.read_point_cloud(self.pcd_path)
        except Exception as e:
             raise ValueError(f"Fehler beim Lesen der Datei: {e}")
        
        if pcd.is_empty():
            raise ValueError(f"Punktwolke leer/nicht gefunden: {self.pcd_path}")

        self.log(f"Erstelle Voxel-Grid...")
        self.voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=self.voxel_size)
        self.origin = self.voxel_grid.origin
        self.log(f"Voxel-Grid Ursprung gefunden bei: {self.origin}")
        
        voxels = self.voxel_grid.get_voxels()
        self.occupied_indices = set((v.grid_index[0], v.grid_index[1], v.grid_index[2]) for v in voxels)
        self.log(f"{len(self.occupied_indices)} belegte Voxel gefunden.")

    def check_clearance(self, vx, vy, vz):
        needed_clearance_voxels = int(np.ceil(self.robot_height_clearance / self.voxel_size))
        for i in range(1, needed_clearance_voxels + 1):
            if (vx, vy, vz + i) in self.occupied_indices:
                return False
        return True

    def identify_walkable_nodes(self):
        self.log("Analysiere Durchgangshöhen (Initial)...")
        for (vx, vy, vz) in self.occupied_indices:
            if self.check_clearance(vx, vy, vz):
                self.walkable_nodes.add((vx, vy, vz))

        initial_count = len(self.walkable_nodes)
        self.log(f"{initial_count} Knoten direkt aus Punktwolke erkannt.")
        self.log("Starte Gap-Filling für zusammenhängenden Boden...")
        self._fill_gaps(iterations=2)
        
        filled_count = len(self.walkable_nodes)
        self.log(f"{filled_count - initial_count} Lücken gefüllt. Total: {filled_count} Knoten.")

        self.xy_column_map = {}
        for (vx, vy, vz) in self.walkable_nodes:
            if (vx, vy) not in self.xy_column_map:
                self.xy_column_map[(vx, vy)] = []
            self.xy_column_map[(vx, vy)].append(vz)

    def _fill_gaps(self, iterations=1):
        for _ in range(iterations):
            new_nodes = set()
            current_nodes = list(self.walkable_nodes)
            for (cx, cy, cz) in current_nodes:
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        nx, ny, nz = cx + dx, cy + dy, cz
                        candidate = (nx, ny, nz)
                        if candidate in self.walkable_nodes or candidate in self.occupied_indices: continue
                        floor_neighbors = 0
                        for ndx in [-1, 0, 1]:
                            for ndy in [-1, 0, 1]:
                                if ndx == 0 and ndy == 0: continue
                                if (nx+ndx, ny+ndy, nz) in self.walkable_nodes:
                                    floor_neighbors += 1
                        if floor_neighbors >= 3:
                            if self.check_clearance(nx, ny, nz):
                                new_nodes.add(candidate)
            if not new_nodes: break
            self.walkable_nodes.update(new_nodes)

    def build_graph(self):
        self.log("Erstelle Graph...")
        neighbor_offsets = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for current_node in self.walkable_nodes:
            cx, cy, cz = current_node
            self.G.add_node(current_node, type="floor")
            for dx, dy in neighbor_offsets:
                nx_idx, ny_idx = cx + dx, cy + dy
                if (nx_idx, ny_idx) in self.xy_column_map:
                    for nz in self.xy_column_map[(nx_idx, ny_idx)]:
                        delta_z_meters = (nz - cz) * self.voxel_size
                        abs_delta_z = abs(delta_z_meters)
                        dist_xy = np.sqrt(dx**2 + dy**2) * self.voxel_size
                        weight = None
                        edge_type = None
                        if abs_delta_z <= self.min_stair_height:
                            weight = np.sqrt(dist_xy**2 + delta_z_meters**2)
                            edge_type = "flat"
                        elif self.min_stair_height < abs_delta_z <= self.max_stair_height:
                            real_dist = np.sqrt(dist_xy**2 + delta_z_meters**2)
                            weight = real_dist * 3.0
                            edge_type = "stair"
                            self.G.nodes[current_node]['type'] = 'stair_access' 
                        if weight is not None:
                            self.G.add_edge(current_node, (nx_idx, ny_idx, nz), weight=weight, type=edge_type)

    def save_graph(self, filename):
        data_packet = {"graph": self.G, "origin": self.origin, "voxel_size": self.voxel_size}
        try:
            with open(filename, 'wb') as f:
                pickle.dump(data_packet, f)
            self.log(f"Graph + Metadaten gespeichert unter {filename}")
        except Exception as e:
            self.log(f"[ERROR] Fehler beim Speichern: {e}")

# --- Der ROS 2 Node Wrapper ---
class PcdToGraphNode(Node):
    def __init__(self):
        super().__init__('pcd_to_graph_node')
        
        # Parameter deklarieren (mit Default-Werten)
        self.declare_parameter('pcd_path', 'environment.pcd')
        self.declare_parameter('voxel_size_cm', 5.0)
        self.declare_parameter('max_step_height_cm', 25.0)
        
    def run_processing(self):
        # Parameter abrufen
        pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        voxel_size_cm = self.get_parameter('voxel_size_cm').get_parameter_value().double_value
        max_step_height_cm = self.get_parameter('max_step_height_cm').get_parameter_value().double_value
        
        # Pfad expandieren (wichtig für ~)
        pcd_path = os.path.abspath(os.path.expanduser(pcd_path))
        
        # Umrechnung
        voxel_size_m = voxel_size_cm / 100.0
        max_step_height_m = max_step_height_cm / 100.0
        
        self.get_logger().info(f"Starte Verarbeitung: {pcd_path}")
        self.get_logger().info(f"Config: Voxel={voxel_size_cm}cm, MaxStep={max_step_height_cm}cm")

        try:
            # Logik-Klasse instanziieren und ROS-Logger übergeben
            processor = VoxelNavigationGraph(
                pcd_path=pcd_path, 
                voxel_size=voxel_size_m, 
                max_stair_height=max_step_height_m,
                logger=self.get_logger()
            )
            processor.load_and_voxelize()
            processor.identify_walkable_nodes()
            processor.build_graph()
            
            output_filename = f"nav_graph_step{int(max_step_height_cm)}_voxel{int(voxel_size_cm)}.pkl"
            processor.save_graph(output_filename)
            
        except Exception as e:
            self.get_logger().error(f"Abbruch durch Fehler: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PcdToGraphNode()
    
    # Da es ein "One-Shot" Script ist, rufen wir die Funktion direkt auf
    # und spinnen nicht endlos.
    node.run_processing()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
