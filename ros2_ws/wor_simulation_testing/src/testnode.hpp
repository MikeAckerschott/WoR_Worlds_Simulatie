#ifndef TESTNODE_HPP_
#define TESTNODE_HPP_

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "msg_srv/msg/robot_command.hpp"


class TestNode : public rclcpp::Node
{
public:
    TestNode();

    virtual ~TestNode() = default;

    sensor_msgs::msg::JointState jointStateMessage_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
    rclcpp::Subscription<msg_srv::msg::RobotCommand>::SharedPtr robotCommandSub_;

    void publishJointState();
    void handleSubRobotCommand(const msg_srv::msg::RobotCommand::SharedPtr msg);

private:
};

#endif /* TESTNODE_HPP_ */