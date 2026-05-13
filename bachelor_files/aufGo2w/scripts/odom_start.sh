#!/bin/bash

# --- AUTO-FIX: Rechte prüfen und Gruppe wechseln ---
if ! groups | grep -q "docker"; then
    echo "⚠️  Keine Docker-Rechte gefunden. Wechsle Gruppe..."
    exec sg docker -c "$0"
    exit 1
fi

# --- KONFIGURATION ---
CONTAINER_NAME="humble_dev_container"
DOCKER_WS="/home/unitree/ros2_humble_docker_ws"
LAUNCH_HELPER="$HOME/launch_docker_humble.sh"
HOST_ROS_DISTRO="foxy" 

# HIER ANGEPASST: Nutzt die reine Odometrie-Launch-Datei
LAUNCH_FILE="lidarslam_odom.launch.py"
# ---------------------

echo "-------------------------------------------------------"
echo "🤖 Unitree Go2: Reine Lidar-Odometrie (Ohne Mapping/RMCL)"
echo "-------------------------------------------------------"

# 1. Container-Check (Robust)
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "❌ FEHLER: Container '$CONTAINER_NAME' läuft nicht!"
    echo "🔄 Versuche, ihn automatisch zu starten..."
    
    if [ -f "$LAUNCH_HELPER" ]; then
        $LAUNCH_HELPER
    else
        echo "❌ Startskript $LAUNCH_HELPER nicht gefunden!"
        exit 1
    fi
    
    sleep 3
    if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "❌ Start fehlgeschlagen. Abbruch."
        exit 1
    fi
fi

# 2. Hardware-Treiber auf dem HOST starten
if [ -f "/opt/ros/$HOST_ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$HOST_ROS_DISTRO/setup.bash
fi

echo "📦 [Host] Starte Unitree SDK & Lidar..."
ros2 run unitree_ros2_example go2w_imu eth0 &
PID_IMU=$!
ros2 run unitree_ros2_example go2w_cmd_vel eth0 &
PID_CMD=$!
ros2 launch hesai_ros_driver start.py &
PID_LIDAR=$!

echo "📦 [Host] Setze Statische Transformationen..."
ros2 run tf2_ros static_transform_publisher 0.1384 0.0 0.1284 1.5708 0 0 base_link hesai_lidar_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 body base_link &

echo "⏳ Warte 5 Sek. auf Sensor-Initialisierung..."
sleep 5

# 3. Odometrie im DOCKER starten
echo "🐳 [Docker] Starte lidarslam_odom im Container..."
echo "ℹ️  Nutze ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0 (Standard)}"

docker exec -i -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID $CONTAINER_NAME bash -c "
    source /opt/ros/humble/install/setup.bash && \
    source $DOCKER_WS/install/setup.bash && \
    ros2 launch lidarslam $LAUNCH_FILE
" &

SLAM_PID=$!

echo "-------------------------------------------------------"
echo "✅ ALLES GESTARTET!"
echo "ℹ️  Die reine Odometrie läuft. Es wird keine Karte gespeichert."
echo "ℹ️  Drücke STRG+C zum sauberen Beenden."
echo "-------------------------------------------------------"

# --- END-FUNKTION ---
function finish {
    trap '' SIGINT SIGTERM
    echo ""
    echo "🛑 STRG+C erkannt! Fahre herunter..."
    
    # Hartes Killen aller Prozesse (verhindert das 'Zombie-Node' Cache Problem)
    echo "👋 Beende alle Prozesse hart..."
    
    # Killt den Docker-Exec Prozess
    kill -9 $SLAM_PID 2>/dev/null
    
    # Sicherheitshalber: Räumt alle hängengebliebenen ROS 2 Nodes im Docker auf
    docker exec -i $CONTAINER_NAME bash -c "pkill -9 -f scanmatcher_node" 2>/dev/null
    
    # Killt die Host Prozesse
    kill -9 $PID_IMU $PID_CMD $PID_LIDAR 2>/dev/null
    kill -9 0
    exit
}

trap finish SIGINT SIGTERM
wait $SLAM_PID
