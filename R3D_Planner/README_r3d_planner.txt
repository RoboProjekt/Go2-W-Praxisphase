-install ros2 
-install colcon "sudo apt install python3-colcon-common-extensions"

-install pip3 "sudo apt install python3-pip"
-install open3d "pip3 install open3d"
-install numpy "pip3 install "numpy<2.0.0" " or pip3 install "numpy<2.0.0" --break-system-packages
-install nav2 for nav2 messages"sudo apt install ros-humble-nav2-msgs"
- install networkx "pip3 install networkx"




-clone repo

run packages

ros2 run r3d_preprocessor pcd_analyser --ros-args -p pcd_path:=/home/basti/Desktop/r3d_ws/src/R3D_Planner/r3d_preprocessor/maps/voxel_05_minhits_7.pcd \
  -p voxel_size_cm:=5.0 \
  -p max_step_height_cm:=25.0 \
  \
  -p min_points_per_sqm:=10.0 \
  -p min_points_per_voxel:=1 \
  -p floor_height_tolerance:=0.03 \
  -p ground_fill:=false \
  \
  -p robot_base_clearance_cm:=10.0 \
  -p robot_narrow_radius_cm:=30.0 \
  -p min_step_height_cm:=5.0 \
  -p robot_radius_cm:=40.0 \
  -p robot_height_cm:=80.0 \
  -p analysis_grid_size_cm:=20.0 \
  -p cluster_gap_threshold_cm:=20.0 \
  -p fill_plane_iterations:=2 \
  -p fill_plane_search_radius:=2 \
  -p fill_plane_min_neighbors:=4
  
ros2 run r3d_preprocessor pcd_server --ros-args -p pcd_path:=/home/basti/Desktop/r3d_ws/src/R3D_Planner/r3d_preprocessor/maps/voxel_05_minhits_7_analysed.pcd

rviz2 with point cloud2 topic "/map_pointcloud"
-set topic durability to "transient local"
-set Style to your prefered display style (note, Flat Squares and Boxes are the most resource intensive, points and spheres are the least resource instensive styles)
-set the size to 0,05m (or your voxel size, if you want), set
-set Color Transform to RGB8 to siplay the traversability as Colours

ros2 run r3d_planner pcd_path_planner --ros-args   -p map_name:=/home/basti/Desktop/r3d_ws/src/R3D_Planner/r3d_preprocessor/maps/voxel_05_minhits_7_analysed.pcd   -p voxel_size_cm:=5.0   -p max_step_height_cm:=25.0   -p min_step_height_cm:=5.0

ros2 run r3d_planner path_test

ros2 run r3d_planner rviz_interface 
-add Marker topic "/planned_path"
-set Reliability "Reliable"
-set Durability "Volatile"
















