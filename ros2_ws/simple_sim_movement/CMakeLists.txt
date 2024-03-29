cmake_minimum_required(VERSION 3.5)
project(simple_sim_movement)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(testing_node src/main.cpp src/testnode.cpp src/mathUtils.cpp)
ament_target_dependencies(testing_node rclcpp tf2_ros geometry_msgs sensor_msgs std_msgs) 

add_executable(cup_publisher src/cup_main.cpp src/CupStatePublisher.cpp src/mathUtils.cpp)
ament_target_dependencies(cup_publisher rclcpp tf2_ros geometry_msgs sensor_msgs std_msgs)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS testing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS cup_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()