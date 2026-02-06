import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
import numpy as np
import math

def create_cloud_xyz32(header, points):
    # Hilfsfunktion zum Erstellen der PC2 Nachricht
    if len(points) == 0:
        return PointCloud2(header=header, height=1, width=0)
        
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * points.shape[0],
        data=points.tobytes()
    )

class ObstacleCliffFilter(Node):
    def __init__(self):
        super().__init__('obstacle_cliff_filter')
        
        self.sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pc_callback, 10)
        
        self.pub_obstacles = self.create_publisher(PointCloud2, '/local/filtered_obstacles', 10)
        self.pub_cliff = self.create_publisher(PointCloud2, '/local/cliff_virtual_wall', 10)
        
        # NEU: Publisher für Stufenerkennung
        self.pub_stair_detect = self.create_publisher(PointStamped, '/stair_detect', 10)

        # --- PARAMETER ---
        # 1. Hindernis-Filter
        self.min_height = 0.05      # Alles unter 5cm ist Boden/Teppich
        self.max_height = 1.0       # Alles über 1m ist Decke
        
        # 2. Stufen-Parameter (Preprocessing sagt max 20cm)
        self.max_step_height = 0.22 # Toleranz etwas über 20cm
        self.min_step_height = 0.06 # Muss höher als min_height sein
        
        # 3. Klippen-Erkennung
        self.cliff_dist = 0.6 
        self.cliff_width = 0.4
        
        # 4. Stufen-Erkennung ROI (Region of Interest)
        self.stair_roi_x_min = 0.7
        self.stair_roi_x_max = 1.1
        self.stair_roi_width = 0.5

    def pc_callback(self, msg):
        # --- SCHRITT 1: DATEN LADEN (Zuerst!) ---
        dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        try:
            cloud_arr = np.frombuffer(msg.data, dtype=np.dtype(dtype_list, align=True))
        except Exception:
             return

        # NumPy Array erstellen
        points = np.zeros((cloud_arr.shape[0], 3), dtype=np.float32)
        points[:, 0] = cloud_arr['x']
        points[:, 1] = cloud_arr['y']
        points[:, 2] = cloud_arr['z']
        
        # NaNs entfernen
        points = points[~np.isnan(points).any(axis=1)]

        # --- SCHRITT 2: TOTZONEN-FILTER (Noise Removal) ---
        # Berechne Distanz in der XY-Ebene
        dist_sq = points[:, 0]**2 + points[:, 1]**2
        
        # Filter: Alles was näher als 0.65m ist, wird ignoriert (Rauschen am Rand der Totzone)
        min_reliable_dist_sq = 0.65**2 
        mask_reliable = dist_sq > min_reliable_dist_sq
        points = points[mask_reliable]

        # --- SCHRITT 3: STUFEN-ERKENNUNG ---
        mask_stair_roi = (
            (points[:, 0] > self.stair_roi_x_min) & 
            (points[:, 0] < self.stair_roi_x_max) & 
            (np.abs(points[:, 1]) < self.stair_roi_width)
        )
        
        mask_step_height = (points[:, 2] > self.min_step_height) & (points[:, 2] < self.max_step_height)
        stair_candidate_points = points[mask_stair_roi & mask_step_height]
        
        is_stair = False
        if len(stair_candidate_points) > 50:
            avg_step_z = np.mean(stair_candidate_points[:, 2])
            avg_step_x = np.mean(stair_candidate_points[:, 0])
            
            stair_msg = PointStamped()
            stair_msg.header = msg.header
            stair_msg.point.x = float(avg_step_x)
            stair_msg.point.y = 0.0
            stair_msg.point.z = float(avg_step_z)
            self.pub_stair_detect.publish(stair_msg)
            
            self.get_logger().info(f"Stufe erkannt! Höhe: {avg_step_z:.3f}m", throttle_duration_sec=1.0)
            is_stair = True

        # --- SCHRITT 4: HINDERNIS-OUTPUT ---
        mask_obstacle = (points[:, 2] > self.min_height) & (points[:, 2] < self.max_height)
        
        if is_stair:
            # Stufe erkannt -> Ignoriere Punkte im Stufenbereich, die niedrig genug sind
            mask_ignorable_step = mask_stair_roi & (points[:, 2] < self.max_step_height)
            mask_final_obstacle = mask_obstacle & (~mask_ignorable_step)
        else:
            mask_final_obstacle = mask_obstacle

        obstacle_points = points[mask_final_obstacle]

        if len(obstacle_points) > 0:
            self.pub_obstacles.publish(create_cloud_xyz32(msg.header, obstacle_points))
        else:
            self.pub_obstacles.publish(create_cloud_xyz32(msg.header, np.zeros((0, 3), dtype=np.float32)))

	# --- SCHRITT 5: KLIPPEN-ERKENNUNG (Korrigiert für Totzone) ---
        
        # WICHTIG: Wir können nur dort nach Boden suchen, wo der Sensor etwas sieht!
        # Deine Totzone ist ca 0.6m. Der Filter oben löscht alles < 0.65m.
        # Also prüfen wir den Bereich von 0.7m bis 1.2m auf Boden.
        
        check_start_x = 0.7
        check_end_x = 1.2  # Wir schauen 50cm weit in den sichtbaren Bereich
        
        mask_roi_cliff_visible = (
            (points[:, 0] > check_start_x) & (points[:, 0] < check_end_x) & 
            (np.abs(points[:, 1]) < self.cliff_width) & 
            (points[:, 2] < self.min_height)
        )
        
        # Zähle Bodenpunkte im SICHTBAREN Bereich
        ground_points_count = np.sum(mask_roi_cliff_visible)
        
        # Schwellwert anpassen: Da der Bereich weiter weg ist, sind die Punkte weniger dicht.
        # Wir senken den Threshold etwas zur Sicherheit.
        MIN_SAFE_POINTS = 30 
        
        if ground_points_count < MIN_SAFE_POINTS:
            # Wir sehen ab 0.7m keinen Boden mehr -> Klippe oder Loch voraus!
            # Wir bauen die Wand genau an die Grenze der Sichtbarkeit
            
            y_fill = np.linspace(-self.cliff_width, self.cliff_width, 20)
            cliff_wall = np.zeros((20, 3), dtype=np.float32)
            
            # Wand muss bei 0.7m stehen (Beginn des sichtbaren Bereichs)
            cliff_wall[:, 0] = check_start_x 
            cliff_wall[:, 1] = y_fill
            cliff_wall[:, 2] = 0.5
            
            self.pub_cliff.publish(create_cloud_xyz32(msg.header, cliff_wall))
            # Optional: Warnung nur ab und zu loggen, um Spam zu vermeiden
            # self.get_logger().warn(f"Klippe/Loch ab {check_start_x}m erkannt! (Bodenpunkte: {ground_points_count})", throttle_duration_sec=2.0)
        else:
            # Boden ist da, Weg ist sicher
            self.pub_cliff.publish(create_cloud_xyz32(msg.header, np.zeros((0, 3), dtype=np.float32)))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCliffFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
