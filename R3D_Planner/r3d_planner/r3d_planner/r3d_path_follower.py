import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_ros import Buffer, TransformListener
import math
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('r3d_path_follower')

        # --- PARAMETER ---
        self.lookahead_distance = 0.5  # Wie weit schaut der Roboter voraus (Meter)
        self.max_linear_speed = 0.35   # Vorwärtsgeschwindigkeit (m/s)
        self.max_angular_speed = 0.6   # Maximale Drehgeschwindigkeit (rad/s)
        self.stop_distance = 0.6       # Bremsweg vor dynamischen Hindernissen (Meter)
        self.goal_tolerance = 0.2      # Wann gilt das Ziel als erreicht?

        # --- VARIABLEN ---
        self.current_path = []
        self.obstacle_in_front = False

        # --- PUBLISHER & SUBSCRIBER ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.path_sub = self.create_subscription(
            Path, '/global_path', self.path_callback, 10)
            
        self.obstacle_sub = self.create_subscription(
            PointCloud2, '/local/filtered_obstacles', self.obstacle_callback, 10)

        # --- TF (Transformationen) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- CONTROL LOOP ---
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        self.get_logger().info("R3D Path Follower gestartet! Warte auf Pfad...")

    def path_callback(self, msg):
        self.current_path = msg.poses
        self.get_logger().info(f"Neuer Pfad empfangen mit {len(self.current_path)} Punkten.")

    def obstacle_callback(self, msg):
        obstacle_detected = False
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if 0.1 < x < self.stop_distance and abs(y) < 0.3:
                obstacle_detected = True
                break
                
        if obstacle_detected and not self.obstacle_in_front:
            self.get_logger().warn("DYNAMISCHES HINDERNIS! Stoppe...")
        elif not obstacle_detected and self.obstacle_in_front:
            self.get_logger().info("Weg ist wieder frei.")
            
        self.obstacle_in_front = obstacle_detected

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except Exception as e:
            return None, None, None

    def control_loop(self):
        msg = Twist()

        if not self.current_path:
            self.cmd_pub.publish(msg)
            return

        if self.obstacle_in_front:
            self.cmd_pub.publish(msg)
            return

        rx, ry, ryaw = self.get_robot_pose()
        if rx is None:
            return

        goal_x = self.current_path[-1].pose.position.x
        goal_y = self.current_path[-1].pose.position.y
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)
        
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("ZIEL ERREICHT!")
            self.current_path = [] 
            self.cmd_pub.publish(msg) 
            return

        target_point = None
        closest_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(self.current_path):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        # --- NEU: Ziel-Index merken ---
        target_idx = closest_idx
        for i in range(closest_idx, len(self.current_path)):
            px = self.current_path[i].pose.position.x
            py = self.current_path[i].pose.position.y
            dist = math.hypot(px - rx, py - ry)
            
            if dist >= self.lookahead_distance:
                target_point = (px, py)
                target_idx = i
                break
                
        if target_point is None:
            target_point = (goal_x, goal_y)
            target_idx = len(self.current_path) - 1

        # --- NEU: Engstellen-Flag auslesen ---
        is_narrow = (self.current_path[target_idx].pose.orientation.z > 0.5)

        dx = target_point[0] - rx
        dy = target_point[1] - ry
        
        target_yaw = math.atan2(dy, dx)
        
        angle_error = target_yaw - ryaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        # --- NEU: Dynamische Bewegungskontrolle ---
        if is_narrow:
            # Engstellen-Modus (Sehr vorsichtig!)
            max_ang_narrow = 0.2  
            
            if abs(angle_error) > 0.5:
                # NUR auf der Stelle drehen, KEIN Vorwärtsfahren in der Tür!
                msg.linear.x = 0.0 
                msg.angular.z = max(-max_ang_narrow, min(max_ang_narrow, angle_error * 1.5))
            else:
                # Er ist gut ausgerichtet -> Langsam durchfahren
                msg.linear.x = self.max_linear_speed * 0.4 
                msg.angular.z = max(-max_ang_narrow, min(max_ang_narrow, angle_error * 1.5))
        else:
            # Normaler Modus (Grüner Boden)
            msg.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angle_error * 1.5))
            
            if abs(angle_error) > 0.5: 
                msg.linear.x = self.max_linear_speed * 0.3 
            else:
                msg.linear.x = self.max_linear_speed

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
