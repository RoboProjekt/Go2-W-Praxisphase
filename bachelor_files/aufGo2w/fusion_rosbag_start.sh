#!/bin/bash

# ==========================================
# SYSTEM-OPTIMIERUNG (UDP BUFFER)
# ==========================================
# Erhöht den Empfangspuffer des Kernels, um Paketverlust bei 2 Lidars zu vermeiden
echo "🚀 Optimiere Netzwerk-Buffer..."
sudo sysctl -w net.core.rmem_max=8388608 2>/dev/null

# ==========================================
# CLEANUP (Alte Prozesse beenden)
# ==========================================
echo "🔍 Prüfe auf alte Instanzen und räume auf..."
pkill -f "hesai_ros_driver" 2>/dev/null
pkill -f "lidar_fusion_node" 2>/dev/null
pkill -f "go2w_imu" 2>/dev/null
pkill -f "go2w_cmd_vel" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "universal_domain_bridge.py" 2>/dev/null
pkill -f "ros2 bag" 2>/dev/null
sleep 2

# ==========================================
# 1. STARTE PYTHON DOMAIN BRIDGE (0 -> 10)
# ==========================================
echo "🌉 Starte L1 Lidar Domain Bridge (0 -> 10)..."
python3 ~/Domain_Bridge/universal_domain_bridge.py &
sleep 3

# ==========================================
# AB HIER ALLES AUF DOMAIN 10
# ==========================================
export ROS_DOMAIN_ID=10

echo "🔗 Starte Lidar Transforms..."
ros2 run tf2_ros static_transform_publisher 0.1384 0.0 0.1284 1.5708 0 0 base_link hesai_lidar_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 body base_link &
ros2 run tf2_ros static_transform_publisher 0.2 0 0 3.14 0.2181662 3.14 base_link utlidar_lidar &
sleep 3

echo "🐕 Starte Unitree SDK..."
ros2 run unitree_ros2_example go2w_imu eth0 &
ros2 run unitree_ros2_example go2w_cmd_vel eth0 &
sleep 5 

echo "🌪️  Starte Hesai Lidar-Treiber..."
ros2 launch hesai_ros_driver start.py rviz:=false &
sleep 5

echo "🧬 Starte C++ Lidar Fusion Node..."
ros2 run lidar_fusion lidar_fusion_node &
sleep 5

# ==========================================
# 6. ROSBAG AUFNAHME MIT QOS OVERRIDES
# ==========================================
echo "🎒 Starte RosBag Aufnahme..."

mkdir -p ~/Desktop/rotzbag
cd ~/Desktop/rotzbag/

TIMESTAMP=$(date +"%Y_%m_%d_%H_%M_%S")
BAG_NAME="fusion_scan_${TIMESTAMP}"

# Aufzeichnung mit den QoS-Einstellungen für Best-Effort Lidar Daten
ros2 bag record \
    --qos-profile-overrides-path ~/Desktop/fusion_Rosbag_domain_bridge/qos_overrides.yaml \
    -o $BAG_NAME \
    /lidar_fusion/point_cloud \
    /utlidar/cloud \
    /hesai_ros_driver/hesai/lidar_points \
    /utlidar/imu \
    /imu/data \
    /tf \
    /tf_static &

cd ~/Desktop/
sleep 2

# ==========================================
# ABSCHLUSS
# ==========================================
echo "--------------------------------------------------------"
echo "✅ System läuft! Architektur: Domain 0 (L1) -> Bridge -> Domain 10 (Rest + Bag)"
echo "📂 Bag-Ordner: ~/Desktop/rotzbag/$BAG_NAME"
echo "🛑 Zum Beenden und Speichern einfach STRG+C drücken."
echo "--------------------------------------------------------"
wait
