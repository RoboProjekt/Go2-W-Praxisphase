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
# Host-Pfad zum Workspace (für die Berechnung der Dateinamen)
HOST_WS="$HOME/ros2_humble_docker_ws"
LAUNCH_HELPER="$HOME/launch_docker_humble.sh"
LAUNCH_FILE="lidarslam_IMU.launch.py"
HOST_ROS_DISTRO="foxy" 
# ---------------------

echo "-------------------------------------------------------"
echo "🤖 Unitree Go2 SLAM Control - Hybrid Mode"
echo "-------------------------------------------------------"

# 1. Container-Check
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

# 3. SLAM im DOCKER starten
echo "🐳 [Docker] Starte lidarslam im Container..."
echo "ℹ️  Nutze ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0 (Standard)}"

docker exec -i -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID $CONTAINER_NAME bash -c "
    source /opt/ros/humble/install/setup.bash && \
    source $DOCKER_WS/install/setup.bash && \
    ros2 launch lidarslam $LAUNCH_FILE
" &

SLAM_PID=$!

echo "-------------------------------------------------------"
echo "✅ ALLES GESTARTET!"
echo "ℹ️  Drücke STRG+C zum Speichern & Beenden"
echo "-------------------------------------------------------"

# --- END-FUNKTION (Hier liegt die Änderung) ---
function finish {
    trap '' SIGINT SIGTERM
    echo ""
    echo "🛑 STRG+C erkannt! Fahre herunter..."
    
    # 1. Verzeichnis auf dem Host sicherstellen
    HOST_MAPS_DIR="$HOST_WS/maps"
    mkdir -p "$HOST_MAPS_DIR"

    # 2. Nächste freie Nummer finden (map_1.pcd, map_2.pcd, ...)
    COUNT=1
    while [ -f "$HOST_MAPS_DIR/map_${COUNT}.pcd" ]; do
        ((COUNT++))
    done
    MAP_NAME="map_${COUNT}.pcd"
    
    # Pfad im Docker Container (muss identisch zum Mount sein oder kopiert werden)
    MAP_PATH_DOCKER="$DOCKER_WS/maps/$MAP_NAME"
    
    echo "💾 Sende Speicherbefehl für '$MAP_NAME' an Docker..."
    
    # Ordner im Docker erstellen, falls er fehlt
    docker exec $CONTAINER_NAME mkdir -p "$DOCKER_WS/maps"

    # 3. Service Aufruf mit korrektem Parameter 'destination_path'
    timeout 3m docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID $CONTAINER_NAME bash -c "
        source /opt/ros/humble/install/setup.bash && \
        source $DOCKER_WS/install/setup.bash && \
        ros2 service call /map_save lidarslam_msgs/srv/SaveMap \"{destination_path: '$MAP_PATH_DOCKER'}\"
    "
    
    if [ $? -eq 0 ]; then
        echo "✅ Karte erfolgreich als $MAP_NAME gespeichert."
        
        # Optional: Falls Docker-Ordner nicht gemountet ist, Datei auf Host kopieren
        # (Wenn du den Ordner gemountet hast, ist dieser Schritt redundant, schadet aber nicht)
        if [ ! -f "$HOST_MAPS_DIR/$MAP_NAME" ]; then
            echo "📤 Kopiere Karte aus Container auf Host..."
            docker cp $CONTAINER_NAME:$MAP_PATH_DOCKER $HOST_MAPS_DIR/
        fi
        
        echo "📍 Speicherort: $HOST_MAPS_DIR/$MAP_NAME"
    else
        echo "⚠️  Fehler beim Speichern (Service nicht erreichbar oder Timeout)!"
    fi
    
    echo "👋 Beende alle Prozesse hart..."
    kill -9 $SLAM_PID 2>/dev/null
    kill -9 $PID_IMU $PID_CMD $PID_LIDAR 2>/dev/null
    kill -9 0
    exit
}

trap finish SIGINT SIGTERM
wait $SLAM_PID
