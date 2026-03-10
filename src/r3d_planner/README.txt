R3D_Planner is a package for real 3d-path planning. It uses a preprocessor and live path planning to make a usable and adaptive Navigation Package for 3d applications. The goal was to make it possible to traverse multiple stories with stairs. It was developed for use with a Unitree Go2W but can be used with any Robot that is capable of climbing stairs.

# Loads the planner with a standart map called nav_graph.pkl
ros2 run r3d_planner global_planner

# For a specified map in r3d_preprocessors/maps run:
ros2 run r3d_planner global_planner --ros-args -p map_name:=test_map.pkl

# Else run:
ros2 run r3d_planner global_planner --ros-args -p map_dir:=/home/user/path/ -p map_name:=test_map.pkl

# Filters height (min_height and max_height for obstacles -> min_height so that a rug is not an obstacle and max_height, so that the robot doesn't stop before an Object it can traverse underneath) and cliff detection so that the robot sees a cliff and doesn't drive over the edge
ros2 run r3d_planner local_filter
