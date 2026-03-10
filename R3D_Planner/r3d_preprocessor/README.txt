R3D_Planner is a package for real 3d-path planning. It uses a preprocessor and live path planning to make a usable and adaptive Navigation Package for 3d applications. The goal was to make it possible to traverse multiple stories with stairs. It was developed for use with a Unitree Go2W but can be used with any Robot that is capable of climbing stairs.

# To run the preprocessor and convert a 3d Lidarscan in .pcd format to a .pkl run the following command. You can find a list of all available Filters and Ros Args etc. in the Filters.txt file.
ros2 run r3d_preprocessor pcd_to_graph

# To publish the .pkl you can run the publisher. It is going to load the Marker and edge graphs of the .pkl onto the ros topics /r3d_global_voxel_map and /r3d_global_graph_edges. The green Blocks represent the ground and can always be traversed, the yellow Boxes have been identified as steps. The red edges connecting the yellow Boxes are representations of which Box is connected to which in order to give the planner information about where the stairs lead. You need to set the fixed frame to /map in order for it to work
ros2 run r3d_preprocessor voxel_map_publisher --ros-args -p graph_path:=nav_graph_step25_voxel10.pkl

# To publish the original 3D scan in .pcd Format
ros2 run r3d_preprocessor pcd_server --ros-args -p pcd_path:=~/Desktop/R3D-Planner/ros2_r3d_planner_ws/src/r3d_preprocessor/maps/environment.pcd
