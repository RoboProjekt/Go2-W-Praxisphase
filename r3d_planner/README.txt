# Loads the planner with a standart map called graph.pkl
ros2 run r3d_planner global_planner

# For a specified map in r3d_preprocessors/maps run:
ros2 run r3d_planner global_planner --ros-args -p map_name:=test_map.pkl

# Else run:
ros2 run r3d_planner global_planner --ros-args -p map_dir:=/home/user/path/ -p map_name:=test_map.pkl

# Filters height (min_height and max_height for obstacles -> min_height so that a rug is not an obstacle and max_height, so that the robot doesn't stop before an Object it can traverse underneath) and cliff detection so that the robot sees a cliff and doesn't drive over the edge
ros2 run r3d_planner local_filter
