#include "communicatorNode.hpp"

CommunicatorNode::CommunicatorNode() : Node("communicator_node")
{

    robotCommandPub_ = this->create_publisher<msg_srv::msg::RobotCommand>("robot_command", 10);

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
}

CommunicatorNode::~CommunicatorNode()
{
}

void CommunicatorNode::sendCommand(std::string command)
{
    msg_srv::msg::RobotCommand msg;
    msg.command = command;
    robotCommandPub_->publish(msg);
}
