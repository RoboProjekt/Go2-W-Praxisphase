R3D_Planner is a package for real 3d-path planning. It uses a preprocessor and live path planning to make a usable and adaptive Navigation Package for 3d applications. The goal was to make it possible to traverse multiple stories with stairs. It was developed for use with a Unitree Go2W but can be used with any Robot that is capable of climbing stairs.

# To run the preprocessor and convert a 3d Lidarscan in .pcd format to a .pkl run the following command. You can change the map it's supposed to load, the maximum step size oft the stairs (or other Objects that need to be traversed) and finally you can set the voxel size. Be careful with pre voxelised maps. The converter will fill gaps in the floor, but only if it has at least 3 neighbours on the same level.
ros2 run r3d_preprocessor pcd_to_graph --ros-args -p pcd_path:=/home/bauya/maps/map1.pcd -p voxel_size_cm:=10.0

# To publish the .pkl you can run the publisher. It is going to load the Marker and edge graphs of the .pkl onto the ros topics /r3d_global_voxel_map and /r3d_global_graph_edges. The green Blocks represent the ground and can always be traversed, the yellow Boxes have been identified as steps. The red edges connecting the yellow Boxes are representations of whoch Box is connected to which in order to give the planner information about where the stairs lead. You need to set the fixed frame to /map inorder for it to work
ros2 run r3d_preprocessor voxel_map_publisher --ros-args -p graph_path:=nav_graph_step25_voxel10.pkl

