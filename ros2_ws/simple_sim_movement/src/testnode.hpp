#ifndef TESTNODE_HPP_
#define TESTNODE_HPP_

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "mathUtils.hpp"


class TestNode : public rclcpp::Node
{
public:
    TestNode();

    virtual ~TestNode() = default;

    sensor_msgs::msg::JointState joint_state_message_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    void publish_joint_state();
    void timerCallback();

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* TESTNODE_HPP_ */