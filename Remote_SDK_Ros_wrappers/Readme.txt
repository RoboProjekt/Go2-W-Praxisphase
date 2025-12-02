Ported to Ros2 Humble

(For Running SDK on Desktop PC)

There are two newly created ros funtions:
-go2w_driver: listenes to /cmd_vel topic and parses the command velocities to the SDK for Movement
-go2w_imu: listens to SDK IMU data and publishes datastream to /imu topic (full IMU data under /imu/data)

Download and install the Unitree_SDK. After you have installed it, copy the go2w folder to "path/to/unitree_ros2/example/src/src/" and exchange the CMakeLists under "/unitree_ros2/example/src/" with the new one. (Only if you want to run it on Ubuntu 22.04 with Ros Humble. The Ros files are compatible with Ros2 Foxy as well so you can also use them under Ubuntu 20.04).

These files have been created with a remote SDK call in mind (We wanted to run the SDK on our Computer). For use with Ros2 Foxy under Ubuntu 20 please check the other Folder labeled Local_SDK_Ros_wrappers

 

both can be launched with: ros2 launch unitree_ros2_example go2w_driver <connection>

ex.: ros2 launch unitree_ros2_example go2w_driver eno1

If you have multiple sourced SDK installations you can also launch with:
path/to/install/unitree_ros2_example/lib/unitree_ros2_example/go2w_driver eno1

or: cd path/to/install/unitree_ros2_example/lib/unitree_ros2_example
    ./go2w_driver eno1

If you want to use the normal SDK Functions and not the ros examples you can find all executables under "path/to/build/unitree_sdk2/bin" and run the executable using ./executable <connection>
