#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "testnode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    TestNode test_node;
    std::cout << "test_node created" << std::endl;

    //publish every 10 ms
    rclcpp::WallRate loop_rate(100);

    while (rclcpp::ok())
    {
        test_node.publish_joint_state();
        rclcpp::spin_some(test_node.get_node_base_interface());
        loop_rate.sleep();
        std::cout<<"test_node published"<<std::endl;
    }

    rclcpp::shutdown();
    return 0;
}