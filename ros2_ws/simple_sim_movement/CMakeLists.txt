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
find_package(msg_srv REQUIRED)

set(SIM_NODE_DIR "src/sim_node")
set(UTILS_DIR "src/utils")
set(CLI_NODE_DIR "src/cli_node")

set(SIM_NODE_SRC
  "${SIM_NODE_DIR}/main.cpp"
  "${SIM_NODE_DIR}/testnode.cpp"
  "${SIM_NODE_DIR}/commandParser.cpp"
)

set(UTILS_SRC
  "${UTILS_DIR}/servoUtils.cpp"
  "${UTILS_DIR}/mathUtils.cpp"
)

set(CLI_NODE_SRC
  "${CLI_NODE_DIR}/mainCommunicatorNode.cpp"
  "${CLI_NODE_DIR}/communicatorNode.cpp"
)

add_executable(testing_node ${SIM_NODE_SRC} ${UTILS_SRC})
ament_target_dependencies(testing_node rclcpp tf2_ros geometry_msgs sensor_msgs msg_srv)

add_executable(cli_node ${CLI_NODE_SRC})
ament_target_dependencies(cli_node rclcpp tf2_ros geometry_msgs sensor_msgs msg_srv)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS testing_node cli_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()