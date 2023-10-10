#include "testnode.hpp"

TestNode::TestNode() : Node("test_node")
{
    jointStatePub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    jointStateMessage_.name = {
        "base_link2turret",
        "turret2upperarm",
        "upperarm2forearm",
        "forearm2wrist",
        "wrist2hand",
        "gripper_left2hand",
        "gripper_right2hand"};
    jointStateMessage_.position = {
        0, // base
        0, // shoulder
        0, // elbow
        0, // wrist
        0, // hand
        0, // gripper 01
        0, // gripper 02
    };

    lastRobotPosition_.name = {
        "base_link2turret",
        "turret2upperarm",
        "upperarm2forearm",
        "forearm2wrist",
        "wrist2hand",
        "gripper_left2hand",
        "gripper_right2hand"};
    lastRobotPosition_.position = {
        0, // base
        0, // shoulder
        0, // elbow
        0, // wrist
        0, // hand
        0, // gripper 01
        0, // gripper 02
    };
    robotCommandSub_ = this->create_subscription<msg_srv::msg::RobotCommand>("robot_command", 10, std::bind(&TestNode::handleSubRobotCommand, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestNode::timerCallback, this));
}

void TestNode::publishJointState()
{
    jointStateMessage_.header.stamp = now();     // set time
                                                 // fill msg
    jointStatePub_->publish(jointStateMessage_); // send

    lastRobotPosition_ = jointStateMessage_;
}

void TestNode::handleSubRobotCommand(const msg_srv::msg::RobotCommand::SharedPtr msg)
{
    std::string messageString = msg->command;

    bool isValid = true;
    auto command = commandParser_.parseCompleteCommand(messageString, isValid);

    if (!isValid)
    {
        std::cout << "Invalid command" << std::endl;
        return;
    }

    commandParser_.calculateRealDuration(command, lastRobotPosition_);

    rclcpp::Clock::SharedPtr clock = get_clock();
    rclcpp::Time current_time = clock->now();

    // Convert the time to milliseconds
    lastActionTime_ = current_time.nanoseconds() / 1000000;
    command.startTime = lastActionTime_;
    this->currentCommand_ = command;

    std::cout << "finsihed parsing command" << std::endl;
}

void TestNode::timerCallback()
{
    rclcpp ::Clock::SharedPtr clock = get_clock();
    rclcpp::Time current_time = clock->now();
    unsigned long long currentTime = current_time.nanoseconds() / 1000000;

    if (currentTime > currentCommand_.duration + currentCommand_.startTime)
    {
        return;
    }

    std::cout << "moving robotic arm" << std::endl;

    int timePassed = currentTime - lastActionTime_;

    std::cout << "time passed: " << timePassed << std::endl;
    for (int i = 0; i < currentCommand_.servoCommands.size(); ++i)
    {
        auto servoCommand = currentCommand_.servoCommands[i];

        std::cout << "servo channel: " << servoCommand.channel << std::endl;
        std::cout << "servo speed: " << servoCommand.speedAnglePerSecond << std::endl;
        std::cout << "servo position: " << jointStateMessage_.position[servoCommand.channel] << std::endl;

        jointStateMessage_.position[servoCommand.channel] = Utils::MathUtils::toRadians((servoCommand.speedAnglePerSecond * timePassed) / 1000) + jointStateMessage_.position[servoCommand.channel];
        std::cout << "new servo position: " << jointStateMessage_.position[servoCommand.channel] << std::endl;
    }

    current_time = clock->now();
    lastActionTime_ = current_time.nanoseconds() / 1000000;
    publishJointState();
    std::cout << "published!" << std::endl;
}