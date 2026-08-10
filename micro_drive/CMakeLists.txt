cmake_minimum_required(VERSION 3.8)
project(micro_drive)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(in_place_turning_experiment_node src/in_place_turning_experiment_node.cpp)
ament_target_dependencies(in_place_turning_experiment_node
  rclcpp sensor_msgs geometry_msgs std_srvs tf2 tf2_ros)

add_executable(imu_bias_observer src/imu_bias_observer.cpp)
ament_target_dependencies(imu_bias_observer
        rclcpp
        std_msgs
        nav_msgs
        sensor_msgs
        tf2_ros
        tf2
        tf2_geometry_msgs
        )
target_link_libraries(imu_bias_observer "${cpp_typesupport_target}")

add_executable(imu_bias_compensator_node src/imu_bias_compensator_node.cpp)
ament_target_dependencies(imu_bias_compensator_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        tf2_ros
        tf2
        tf2_geometry_msgs
        )
target_link_libraries(imu_bias_compensator_node "${cpp_typesupport_target}")

install(TARGETS imu_bias_compensator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS imu_bias_observer
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  in_place_turning_experiment_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
