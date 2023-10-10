#ifndef COMMUNICATORNODE_HPP_
#define COMMUNICATORNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "msg_srv/msg/robot_command.hpp"

/**
 * @class CommunicatorNode
 * @brief Node that sends commands to the high level driver
 * @details Currently supports commands: singleServo, multiServo, stop, programmedPosition
 */

class CommunicatorNode : public rclcpp::Node
{

public:
    /**
     * @brief Constructor
     */
    CommunicatorNode();

    /**
     * @brief Destructor
     */
    ~CommunicatorNode();

    void sendCommand(std::string command);

private:
    rclcpp::Publisher<msg_srv::msg::RobotCommand>::SharedPtr robotCommandPub_;
};

#endif /* COMMUNICATORNODE_HPP_ */