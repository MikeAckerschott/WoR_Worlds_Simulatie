#include "testnode.hpp"

TestNode::TestNode() : Node("test_node")
{
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_message_.name = {
        "base_link2turret",
        "turret2upperarm",
        "upperarm2forearm",
        "forearm2wrist",
        "wrist2hand",
        "gripper_left2hand",
        "gripper_right2hand"};
    joint_state_message_.position = {
        0, // base
        0, // shoulder
        0, // elbow
        0, // wrist
        0, // hand
        0, // gripper 01
        0, // gripper 02
    };
    joint_state_pub_->publish(joint_state_message_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestNode::timerCallback, this));

    robot_command_sub_ = this->create_subscription<msg_srv::msg::RobotCommand>(
        "robot_command", 10, std::bind(&TestNode::handle_robot_command, this, std::placeholders::_1));
}

void TestNode::publish_joint_state()
{
    joint_state_message_.header.stamp = now();       // set time
                                                     // fill msg
    joint_state_pub_->publish(joint_state_message_); // send
}

void TestNode::timerCallback()
{

    if (commandQueue.size() > 0)
    {
        CommandParser::CompleteCommand command = commandQueue.front();
        bool currentCommandFinished = true;

        for (int i = 0; i < command.servoCommands.size(); i++)
        {
            CommandParser::ServoCommand servoCommand = command.servoCommands[i];

            double angleDestination = ServoUtils::pwmToDegrees(servoCommand.pulseWidth);
            double angleCurrent = Utils::MathUtils::toDegrees(joint_state_message_.position[servoCommand.channel]);

            std::cout<<"angleDestination: "<<angleDestination<<std::endl;
            std::cout<<"angleCurrent: "<<angleCurrent<<std::endl;

            double angleDifference = angleDestination - angleCurrent;
            double anglePerSecond = servoCommand.speedAnglePerSecond;

            std::cout<<"angleDifference: "<<abs(angleDifference)<<std::endl;
            std::cout<<"anglePerSecond: "<<abs(anglePerSecond)<<std::endl;

            if (abs(angleDifference) > abs(anglePerSecond * 0.01))
            {
                joint_state_message_.position[servoCommand.channel] += Utils::MathUtils::toRadians(servoCommand.speedAnglePerSecond * 0.01);
                std::cout<<"moving servo "<<servoCommand.channel<<" to "<<joint_state_message_.position[servoCommand.channel]<<std::endl;
                currentCommandFinished = false;
            }
            else
            {
                joint_state_message_.position[servoCommand.channel] = Utils::MathUtils::toRadians(angleDestination);
            }
        }
        if (currentCommandFinished)
        {
            commandQueue.pop();
        }
    }

    publish_joint_state();
}

void TestNode::handle_robot_command(const msg_srv::msg::RobotCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->command.c_str());
    bool isValid = true;

    CommandParser::CompleteCommand command = parser.parseCompleteCommand(msg->command, isValid);
    parser.calculateRealDuration(command, joint_state_message_);

    if (isValid)
    {
        std::cout<<"added command to queue"<<std::endl;
        commandQueue.push(command);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Invalid command");
    }
}