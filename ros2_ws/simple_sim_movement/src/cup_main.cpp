#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "CupStatePublisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // create shared ptr testnode
    auto test_node = std::make_shared<CupStatePublisher>();

    std::cout << "test_node created FOR CUP" << std::endl;

    rclcpp::spin(test_node);

    rclcpp::shutdown();
    return 0;
}