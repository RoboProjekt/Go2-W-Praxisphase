#!/bin/bash

# --- PFAD-ANPASSUNGEN FÜR DOCKER AUF DEM GO2W ---
WS_DIR="/home/unitree/ros2_humble_docker_ws"
MAPS_DIR="$WS_DIR/src/r3d_preprocessor/maps"

# --- KONFIGURATION ---
DEFAULT_PCD="voxel_05_minhits_7.pcd"
DEFAULT_PKL="voxel_05_minhits_7_nofill_clear10_narrow30.pkl" 

PCD_FILE=${1:-$DEFAULT_PCD}
PKL_FILE=${2:-$DEFAULT_PKL}

echo "------------------------------------------------"
echo "R3D Planner Launch System (Go2W Docker Edition)"
echo "------------------------------------------------"
echo "PCD Datei (Optik):  $PCD_FILE"
echo "PKL Datei (Logik):  $PKL_FILE"
echo "Pfad:               $MAPS_DIR"
echo "------------------------------------------------"

if [ ! -f "$MAPS_DIR/$PCD_FILE" ]; then
    echo "ERROR: PCD Datei nicht gefunden in $MAPS_DIR"
    exit 1
fi
if [ ! -f "$MAPS_DIR/$PKL_FILE" ]; then
    echo "ERROR: PKL Datei nicht gefunden in $MAPS_DIR"
    exit 1
fi

# ROS 2 Base und Workspace sourcen
source /opt/ros/humble/install/setup.bash
source $WS_DIR/install/setup.bash

# Trap: Beendet alle Hintergrundprozesse beim Skript-Abbruch (STRG+C)
trap "kill 0" EXIT

# 1. Voxel Map Publisher
echo "[1/6] Starte Voxel Map Publisher..."
ros2 run r3d_preprocessor voxel_map_publisher --ros-args -p graph_path:=$MAPS_DIR/$PKL_FILE &

# 2. PCD Server
echo "[2/6] Starte PCD Server..."
ros2 run r3d_preprocessor pcd_server --ros-args -p pcd_path:=$MAPS_DIR/$PCD_FILE &

# 3. Global Planner
echo "[3/6] Starte Global Planner..."
ros2 run r3d_planner global_planner --ros-args -p map_name:=$MAPS_DIR/$PKL_FILE &

# 4. Local Filter
echo "[4/6] Starte Local Filter..."
ros2 run r3d_planner local_filter &

# 5. Path Follower
echo "[5/6] Starte Path Follower..."
ros2 run r3d_planner path_follower &

# 6. RViz 3D Interface 
echo "[6/6] Starte RViz 3D Interface für Webserver..."
ros2 run r3d_planner rviz_interface &

echo "Warte 3 Sekunden auf Node-Startup..."
sleep 3

echo "------------------------------------------------"
echo "✅ SYSTEM BEREIT AUF DEM GO2W"
echo "------------------------------------------------"

wait
