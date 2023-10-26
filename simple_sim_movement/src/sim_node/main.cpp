#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "testnode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // create shared ptr testnode
    auto test_node = std::make_shared<TestNode>();

    std::cout << "test_node created" << std::endl;

    rclcpp::spin(test_node);

    rclcpp::shutdown();
    return 0;
}