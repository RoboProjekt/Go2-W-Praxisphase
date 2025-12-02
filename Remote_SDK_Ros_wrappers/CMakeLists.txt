cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencies finden
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED) # Wichtig für cmd_vel
find_package(sensor_msgs REQUIRED) # Wichtig für imu
find_package(unitree_sdk2 REQUIRED) 

# 2. Globale Includes
include_directories(include include/common include/nlohmann)
link_directories(src)

set(DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# --- Standard Beispiele ---
add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(low_level_ctrl_hg src/h1-2/lowlevel/low_level_ctrl_hg.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_low_state_hg src/read_low_state_hg.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)

add_executable(go2_sport_client src/go2/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
target_compile_features(go2_sport_client PRIVATE cxx_std_17)

add_executable(go2_stand_example src/go2/go2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(go2_stand_example PRIVATE cxx_std_17)

add_executable(go2_robot_state_client src/go2/go2_robot_state_client.cpp src/common/motor_crc.cpp) 
target_compile_features(go2_robot_state_client PRIVATE cxx_std_17)

add_executable(g1_arm_sdk_dds_example src/g1/high_level/g1_arm_sdk_dds_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_arm_sdk_dds_example PRIVATE cxx_std_17)

add_executable(g1_arm_action_example src/g1/high_level/g1_arm_action_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_arm_action_example PRIVATE cxx_std_17)

add_executable(g1_dex3_example src/g1/dex3/g1_dex3_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_dex3_example PRIVATE cxx_std_17)

add_executable(g1_loco_client_example src/g1/high_level/loco_client_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_loco_client_example PRIVATE cxx_std_17)

add_executable(g1_ankle_swing_example src/g1/lowlevel/g1_ankle_swing_example.cpp src/common/motor_crc_hg.cpp)
target_compile_features(g1_ankle_swing_example PRIVATE cxx_std_17)

add_executable(g1_audio_client_example src/g1/audio_client/g1_audio_client_example.cpp)
target_compile_features(g1_audio_client_example PRIVATE cxx_std_17)

add_executable(b2w_sport_client src/b2w/b2w_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2w_sport_client PRIVATE cxx_std_17)

add_executable(b2w_stand_example src/b2w/b2w_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2w_stand_example PRIVATE cxx_std_17)

add_executable(b2_stand_example src/b2/b2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2_stand_example PRIVATE cxx_std_17)

add_executable(b2_sport_client src/b2/b2_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2_sport_client PRIVATE cxx_std_17)

# ==========================================
# NEU: Go2W Driver Node Configuration
# ==========================================
add_executable(go2w_driver src/go2w/go2w_driver.cpp)
target_compile_features(go2w_driver PRIVATE cxx_std_17)

# Wir geben ZWEI Pfade hartcodiert an:
# 1. Für unitree/robot/...
# 2. Für dds/topic/... (Das liegt im ddscxx Ordner!)
target_include_directories(go2w_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/home/bauya/ros2_ws/install/unitree_sdk2/include"
  "/home/bauya/ros2_ws/install/unitree_sdk2/include/ddscxx" 
)

# Bibliothek linken
target_link_libraries(go2w_driver
  "/home/bauya/ros2_ws/install/unitree_sdk2/lib/libunitree_sdk2.a"
  "/home/bauya/ros2_ws/install/unitree_sdk2/lib/libddsc.so"
  "/home/bauya/ros2_ws/install/unitree_sdk2/lib/libddscxx.so"
  pthread
)

# ROS 2 Dependencies
ament_target_dependencies(go2w_driver 
  ${DEPENDENCY_LIST}
  geometry_msgs
)
# ==========================================

# ==========================================
# NEU: Go2W IMU Node Configuration
# ==========================================
add_executable(go2w_imu src/go2w/go2w_imu.cpp)
target_compile_features(go2w_imu PRIVATE cxx_std_17)

# Include Directories (Pfade kopiert von go2w_driver)
target_include_directories(go2w_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/home/bauya/ros2_ws/install/unitree_sdk2/include"
  "/home/bauya/ros2_ws/install/unitree_sdk2/include/ddscxx" 
)

# Bibliotheken linken
target_link_libraries(go2w_imu
  "/home/bauya/ros2_ws/install/unitree_sdk2/lib/libunitree_sdk2.a"
  "/home/bauya/ros2_ws/install/unitree_sdk2/lib/libddsc.so"
  "/home/bauya/ros2_ws/install/unitree_sdk2/lib/libddscxx.so"
  pthread
)

# ROS 2 Dependencies
ament_target_dependencies(go2w_imu 
  ${DEPENDENCY_LIST}
  sensor_msgs
  geometry_msgs
)

# ==========================================


# --- Dependencies verlinken (Standard) ---
ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(low_level_ctrl_hg ${DEPENDENCY_LIST})
ament_target_dependencies(g1_low_level_example ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state_hg ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})
ament_target_dependencies(g1_arm_sdk_dds_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_arm_action_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_dex3_example ${DEPENDENCY_LIST})
target_include_directories(g1_dex3_example PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(g1_loco_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_ankle_swing_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_audio_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(b2w_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(b2w_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(b2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(b2_stand_example ${DEPENDENCY_LIST})

# YAML Optional
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    if (${yaml-cpp_VERSION} VERSION_GREATER_EQUAL "0.6")
        add_executable(g1_dual_arm_example src/g1/lowlevel/g1_dual_arm_example.cpp src/common/motor_crc_hg.cpp)
        target_compile_features(g1_dual_arm_example PRIVATE cxx_std_17)
        ament_target_dependencies(g1_dual_arm_example yaml-cpp ${DEPENDENCY_LIST})
        install(TARGETS g1_dual_arm_example DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# Installation
install(TARGETS 
        low_level_ctrl
        low_level_ctrl_hg
        g1_low_level_example
        read_low_state
        read_low_state_hg
        read_motion_state
        read_wireless_controller
        go2_sport_client
        go2_stand_example
        go2w_driver   # <--- Hier
	go2w_imu # <--- Hier
        go2_robot_state_client
        g1_arm_sdk_dds_example
        g1_arm_action_example
        g1_dex3_example
        g1_loco_client_example
        g1_ankle_swing_example
        g1_audio_client_example
        b2w_sport_client
        b2w_stand_example
        b2_stand_example
        b2_sport_client
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
