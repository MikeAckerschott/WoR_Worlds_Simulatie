#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "testnode.hpp"
#include "commandParser.hpp"

// include cin
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto testNode = std::make_shared<TestNode>();
    // spin testNode
    rclcpp::spin(testNode);

    rclcpp::shutdown();
    return 0;
}