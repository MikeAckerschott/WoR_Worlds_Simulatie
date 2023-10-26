#ifndef TESTNODE_HPP_
#define TESTNODE_HPP_

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "msg_srv/srv/pickup_cup.hpp"

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

private:
    void handle_robot_command(const msg_srv::msg::RobotCommand::SharedPtr msg);
    void publish_joint_state();
    void timerCallback();

    void handleCupTransform();

    void initTF2();
    void onClosedGripper();
    void onOpenedGripper();

    void emptyQueue();
    void skipCurrentCommand();

private:
    std::string sim_link_, bot_link_, cup_link_;

    sensor_msgs::msg::JointState joint_state_message_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<msg_srv::msg::RobotCommand>::SharedPtr robot_command_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cupTransformTimer_;
    rclcpp::Client<msg_srv::srv::PickupCup>::SharedPtr pickupCupClient_;

    CommandParser parser;

    std::queue<CommandParser::CompleteCommand> commandQueue;

    geometry_msgs::msg::TransformStamped transform_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
};

#endif /* TESTNODE_HPP_ */