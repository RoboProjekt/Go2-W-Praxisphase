import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
import os
import traceback

class VoxelTraversabilityAnalyzer:
    def __init__(self, pcd_path, voxel_size=0.05, max_stair_height=0.25, logger=None):
        self.pcd_path = pcd_path
        self.voxel_size = voxel_size
        
        # --- ROBOTER DIMENSIONEN ---
        self.robot_radius = 0.40  
        self.narrow_radius = 0.30 
        self.robot_height = 0.80 
        self.robot_base_clearance = 0.10 
        
        # --- TREPPEN PARAMETER ---
        self.min_stair_height = 0.05 
        self.max_stair_height = max_stair_height
        
        # --- BODENERKENNUNG PARAMETER ---
        self.min_points_per_sqm = 10.0 
        self.floor_height_tolerance = 0.02
        self.min_points_per_voxel = 3 
        self.enable_ground_fill = True 
        
        self.analysis_grid_size = 0.20
        self.cluster_gap_threshold = 0.20
        
        # --- LÜCKENFÜLLER PARAMETER ---
        self.fill_plane_iterations = 2
        self.fill_plane_search_radius = 2
        self.fill_plane_min_neighbors = 4
        
        self.voxel_grid = None
        self.occupied_indices = set()
        self.walkable_nodes = set()
        
        self.node_types = {} 
        self.origin = np.array([0.0, 0.0, 0.0])
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

        if self.enable_ground_fill:
            self.log("Starte dichte-basierte Bodenerkennung...")
            pcd = self.augment_floor_from_density(pcd)

        self.log(f"Erstelle Voxel-Grid (Size: {self.voxel_size}m)...")
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=self.voxel_size)
        self.origin = voxel_grid.origin
        
        points = np.asarray(pcd.points)
        indices = np.floor((points - self.origin) / self.voxel_size).astype(int)
        
        voxel_counts = {}
        for idx in indices:
            t_idx = tuple(idx)
            voxel_counts[t_idx] = voxel_counts.get(t_idx, 0) + 1
            
        filtered_voxels = []
        for idx, count in voxel_counts.items():
            if count >= self.min_points_per_voxel:
                filtered_voxels.append(idx)
        
        self.occupied_indices = set(filtered_voxels)
        self.log(f"{len(self.occupied_indices)} belegte Voxel nach Filterung.")

    def augment_floor_from_density(self, pcd):
        points = np.asarray(pcd.points)
        if len(points) == 0: return pcd

        min_bound = points.min(axis=0)
        grid_indices = np.floor((points[:, :2] - min_bound[:2]) / self.analysis_grid_size).astype(int)
        
        grid_dict = {}
        for idx, (ix, iy) in enumerate(grid_indices):
            cell_key = (ix, iy)
            z_val = points[idx, 2]
            if cell_key not in grid_dict: grid_dict[cell_key] = []
            grid_dict[cell_key].append(z_val)
            
        new_points = []
        min_points_in_cell = max(3, int(self.min_points_per_sqm * (self.analysis_grid_size**2)))
        added_count = 0
        
        for (ix, iy), z_values in grid_dict.items():
            if len(z_values) < min_points_in_cell: continue
            z_values = np.array(z_values)
            z_values.sort()
            
            clusters = []
            current_cluster = [z_values[0]]
            for i in range(1, len(z_values)):
                if z_values[i] - z_values[i-1] > self.cluster_gap_threshold: 
                    clusters.append(current_cluster)
                    current_cluster = []
                current_cluster.append(z_values[i])
            clusters.append(current_cluster)
            
            for cluster in clusters:
                if len(cluster) < min_points_in_cell: continue
                cluster = np.array(cluster)
                z_max = cluster.max()
                z_min = cluster.min()
                
                if (z_max - z_min) < ((2 * self.floor_height_tolerance) + 0.02):
                    z_mean = cluster.mean()
                    wx_start = min_bound[0] + ix * self.analysis_grid_size
                    wy_start = min_bound[1] + iy * self.analysis_grid_size
                    fill_step = self.voxel_size / 2.0 
                    for fx in np.arange(wx_start, wx_start + self.analysis_grid_size, fill_step):
                        for fy in np.arange(wy_start, wy_start + self.analysis_grid_size, fill_step):
                            new_points.append([fx, fy, z_mean])
                            added_count += 1
                            
        if len(new_points) > 0:
            all_points = np.vstack((points, np.array(new_points)))
            new_pcd = o3d.geometry.PointCloud()
            new_pcd.points = o3d.utility.Vector3dVector(all_points)
            return new_pcd
        return pcd

    def check_collision_cylinder(self, cx, cy, cz, check_radius):
        rad_vox = int(np.ceil(check_radius / self.voxel_size))
        h_vox = int(np.ceil(self.robot_height / self.voxel_size))
        
        clearance_vox = int(np.floor(self.robot_base_clearance / self.voxel_size))
        start_dz = max(1, clearance_vox) 
        
        for dx in range(-rad_vox, rad_vox + 1):
            for dy in range(-rad_vox, rad_vox + 1):
                if (dx*dx + dy*dy) * (self.voxel_size**2) > (check_radius**2):
                    continue
                nx, ny = cx + dx, cy + dy
                for dz in range(start_dz, h_vox + 1):
                    if (nx, ny, cz + dz) in self.occupied_indices:
                        return False 
        return True

    def identify_walkable_nodes(self):
        self.log(f"1. Initiale Boden-Erkennung (Kollisionscheck)...")
        
        clearance_vox = int(np.floor(self.robot_base_clearance / self.voxel_size))
        start_dz = max(1, clearance_vox)
        
        candidates = []
        for (vx, vy, vz) in self.occupied_indices:
            if (vx, vy, vz + start_dz) not in self.occupied_indices:
                candidates.append((vx, vy, vz))
                
        for (vx, vy, vz) in candidates:
            if self.check_collision_cylinder(vx, vy, vz, self.robot_radius):
                self.walkable_nodes.add((vx, vy, vz))
                self.node_types[(vx, vy, vz)] = "floor"
            elif self.check_collision_cylinder(vx, vy, vz, self.narrow_radius):
                self.walkable_nodes.add((vx, vy, vz))
                self.node_types[(vx, vy, vz)] = "narrow"
                
        initial_count = len(self.walkable_nodes)
        self.log(f"-> {initial_count} Knoten erkannt.")
        
        if self.enable_ground_fill:
            self.log("2. Starte 'Plane Filling' (Nachbarschaft)...")
            self._fill_floor_planes(iterations=self.fill_plane_iterations, search_radius=self.fill_plane_search_radius)
            filled_count = len(self.walkable_nodes)
            self.log(f"-> {filled_count - initial_count} Lücken gefüllt. Total: {filled_count} Knoten.")

        self.xy_column_map = {}
        for (vx, vy, vz) in self.walkable_nodes:
            if (vx, vy) not in self.xy_column_map:
                self.xy_column_map[(vx, vy)] = []
            self.xy_column_map[(vx, vy)].append(vz)

    def _fill_floor_planes(self, iterations=2, search_radius=2):
        for it in range(iterations):
            new_nodes = set()
            current_nodes = list(self.walkable_nodes)
            for (cx, cy, cz) in current_nodes:
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        if dx == 0 and dy == 0: continue
                        nx, ny, nz = cx + dx, cy + dy, cz
                        candidate = (nx, ny, nz)
                        if candidate in self.walkable_nodes or candidate in self.occupied_indices: continue
                        
                        floor_neighbors_count = 0
                        for rdx in range(-search_radius, search_radius + 1):
                            for rdy in range(-search_radius, search_radius + 1):
                                if rdx == 0 and rdy == 0: continue
                                for dz in [-1, 0, 1]:
                                    check_pos = (nx + rdx, ny + rdy, nz + dz)
                                    if check_pos in self.walkable_nodes:
                                        floor_neighbors_count += 1
                        
                        if floor_neighbors_count >= self.fill_plane_min_neighbors:
                            if self.check_collision_cylinder(nx, ny, nz, self.robot_radius):
                                new_nodes.add(candidate)
                                self.node_types[candidate] = "floor"
                            elif self.check_collision_cylinder(nx, ny, nz, self.narrow_radius):
                                new_nodes.add(candidate)
                                self.node_types[candidate] = "narrow"
                                
            if not new_nodes: break
            self.walkable_nodes.update(new_nodes)

    def identify_stairs(self):
        self.log("3. Erkenne Treppen und Stufen...")
        neighbor_offsets = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        stair_count = 0
        for current_node in self.walkable_nodes:
            cx, cy, cz = current_node
            
            for dx, dy in neighbor_offsets:
                nx_idx, ny_idx = cx + dx, cy + dy
                if (nx_idx, ny_idx) in self.xy_column_map:
                    for nz in self.xy_column_map[(nx_idx, ny_idx)]:
                        delta_z_meters = (nz - cz) * self.voxel_size
                        abs_delta_z = abs(delta_z_meters)
                        
                        if self.min_stair_height < abs_delta_z <= self.max_stair_height:
                            self.node_types[current_node] = 'stair_access'
                            stair_count += 1
                            break # Ein Treppen-Nachbar reicht
                            
        self.log(f"-> {stair_count} Treppen-Voxel identifiziert.")

    def save_colored_pcd(self, output_filename):
        self.log("Erstelle farbcodierte Voxel-Punktwolke...")
        obstacles = self.occupied_indices - self.walkable_nodes
        
        points = []
        colors = []
        
        # 1. Hindernisse (Pink)
        for (vx, vy, vz) in obstacles:
            x = self.origin[0] + vx * self.voxel_size + self.voxel_size/2
            y = self.origin[1] + vy * self.voxel_size + self.voxel_size/2
            z = self.origin[2] + vz * self.voxel_size + self.voxel_size/2
            points.append([x, y, z])
            colors.append([1.0, 0.0, 1.0]) # RGB: Pink
            
        # 2. Begehbare Bereiche (Boden, Narrow, Stairs)
        for (vx, vy, vz) in self.walkable_nodes:
            x = self.origin[0] + vx * self.voxel_size + self.voxel_size/2
            y = self.origin[1] + vy * self.voxel_size + self.voxel_size/2
            z = self.origin[2] + vz * self.voxel_size + self.voxel_size/2
            points.append([x, y, z])
            
            n_type = self.node_types.get((vx, vy, vz), "floor")
            if n_type == "floor":
                colors.append([0.0, 1.0, 0.0]) # RGB: Grün
            elif n_type == "narrow":
                colors.append([0.0, 1.0, 1.0]) # RGB: Türkis (Cyan)
            elif n_type == "stair_access":
                colors.append([1.0, 0.8, 0.0]) # RGB: Gelb
                
        # Erstelle Open3D Punktwolke
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(np.array(points))
        out_pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
        
        self.log(f"Speichere PCD mit {len(points)} Voxeln...")
        try:
            o3d.io.write_point_cloud(output_filename, out_pcd)
            self.log(f"✅ Analyse erfolgreich gespeichert unter: {output_filename}")
        except Exception as e:
            self.log(f"[ERROR] Fehler beim Speichern: {e}")


class PcdToGraphNode(Node):
    def __init__(self):
        super().__init__('pcd_to_graph_node')
        self.declare_parameter('pcd_path', 'environment.pcd')
        self.declare_parameter('voxel_size_cm', 5.0)
        self.declare_parameter('max_step_height_cm', 25.0)
        
        self.declare_parameter('min_points_per_sqm', 10.0)
        self.declare_parameter('min_points_per_voxel', 3)
        self.declare_parameter('floor_height_tolerance', 0.02)
        self.declare_parameter('ground_fill', True) 
        
        self.declare_parameter('robot_base_clearance_cm', 10.0) 
        self.declare_parameter('robot_narrow_radius_cm', 30.0) 
        self.declare_parameter('min_step_height_cm', 5.0)
        self.declare_parameter('robot_radius_cm', 40.0)
        self.declare_parameter('robot_height_cm', 80.0)
        self.declare_parameter('analysis_grid_size_cm', 20.0)
        self.declare_parameter('cluster_gap_threshold_cm', 20.0)
        self.declare_parameter('fill_plane_iterations', 2)
        self.declare_parameter('fill_plane_search_radius', 2)
        self.declare_parameter('fill_plane_min_neighbors', 4)
        
    def run_processing(self):
        pcd_path_param = self.get_parameter('pcd_path').get_parameter_value().string_value
        pcd_path = os.path.abspath(os.path.expanduser(pcd_path_param))
        
        try:
            processor = VoxelTraversabilityAnalyzer(
                pcd_path=pcd_path, 
                voxel_size=self.get_parameter('voxel_size_cm').get_parameter_value().double_value / 100.0, 
                max_stair_height=self.get_parameter('max_step_height_cm').get_parameter_value().double_value / 100.0, 
                logger=self.get_logger()
            )
            
            # --- Zuweisung der Parameter ---
            processor.min_points_per_sqm = self.get_parameter('min_points_per_sqm').get_parameter_value().double_value
            processor.min_points_per_voxel = self.get_parameter('min_points_per_voxel').get_parameter_value().integer_value
            processor.floor_height_tolerance = self.get_parameter('floor_height_tolerance').get_parameter_value().double_value
            processor.enable_ground_fill = self.get_parameter('ground_fill').get_parameter_value().bool_value
            
            processor.robot_base_clearance = self.get_parameter('robot_base_clearance_cm').get_parameter_value().double_value / 100.0
            processor.narrow_radius = self.get_parameter('robot_narrow_radius_cm').get_parameter_value().double_value / 100.0
            processor.robot_radius = self.get_parameter('robot_radius_cm').get_parameter_value().double_value / 100.0
            processor.robot_height = self.get_parameter('robot_height_cm').get_parameter_value().double_value / 100.0
            processor.min_stair_height = self.get_parameter('min_step_height_cm').get_parameter_value().double_value / 100.0
            processor.analysis_grid_size = self.get_parameter('analysis_grid_size_cm').get_parameter_value().double_value / 100.0
            processor.cluster_gap_threshold = self.get_parameter('cluster_gap_threshold_cm').get_parameter_value().double_value / 100.0
            processor.fill_plane_iterations = self.get_parameter('fill_plane_iterations').get_parameter_value().integer_value
            processor.fill_plane_search_radius = self.get_parameter('fill_plane_search_radius').get_parameter_value().integer_value
            processor.fill_plane_min_neighbors = self.get_parameter('fill_plane_min_neighbors').get_parameter_value().integer_value
            
            processor.load_and_voxelize()
            processor.identify_walkable_nodes()
            processor.identify_stairs()
            
            # --- DATEINAME ANPASSEN (_analysed) ---
            base_name, ext = os.path.splitext(pcd_path)
            output_path = f"{base_name}_analysed{ext}"
            
            processor.save_colored_pcd(output_path)
            
        except Exception as e:
            self.get_logger().error(f"Abbruch durch Fehler: {e}")
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = PcdToGraphNode()
    node.run_processing()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
