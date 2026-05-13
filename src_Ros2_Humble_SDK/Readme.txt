Ported to Ros2 Humble

(For Running SDK on Desktop PC)

There are two newly created ros funtions:
-go2w_driver: listenes to /cmd_vel topic and parses the command velocities to the SDK for Movement
-go2w_imu: listens to SDK IMU data and publishes datastream to /imu topic (full IMU data under /imu/data)

both can be called with: ros2 launch unitree_ros2_example go2w_driver <connection>

ex.: ros2 launch unitree_ros2_example go2w_driver eno1

If you have multiple sourced SDK installations you can also launch with:
path/to/install/unitree_ros2_example/lib/unitree_ros2_example/go2w_driver eno1

or: cd path/to/install/unitree_ros2_example/lib/unitree_ros2_example
    ./go2w_driver eno1

Both files code is under  "/unitree_ros2/example/src/src/go2w"

CMakeLists under "/unitree_ros2/example/src/" has been Modified to work with Ubuntu 22.04 for Ros Humble.

Please rename this folder to "/src" to avoid possible compilation problems

Compile using colcon build before launching.

If you want to use the normal SDK Functions and not the ros examples you can find all executables under "path/to/build/unitree_sdk2/bin" and run the executable using ./executable <connection>
