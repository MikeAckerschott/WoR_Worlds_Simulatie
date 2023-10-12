#ifndef TESTNODE_HPP_
#define TESTNODE_HPP_

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "../utils/mathUtils.hpp"
#include "../utils/servoUtils.hpp"
#include "msg_srv/msg/robot_command.hpp"
#include "commandParser.hpp"

#include <rclcpp/rclcpp.hpp>
#include <queue>


class TestNode : public rclcpp::Node
{
public:
    TestNode();

    virtual ~TestNode() = default;

    sensor_msgs::msg::JointState joint_state_message_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<msg_srv::msg::RobotCommand>::SharedPtr robot_command_sub_;

    void publish_joint_state();
    void timerCallback();

    CommandParser parser;

    std::queue<CommandParser::CompleteCommand> commandQueue;

private:
    rclcpp::TimerBase::SharedPtr timer_;

    void handle_robot_command(const msg_srv::msg::RobotCommand::SharedPtr msg);
};

#endif /* TESTNODE_HPP_ */