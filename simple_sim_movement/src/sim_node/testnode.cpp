#include "testnode.hpp"

TestNode::TestNode() : Node("test_node"), buffer_(this->get_clock()), listener_(buffer_), broadcaster_(this), sim_link_("sim_link"), bot_link_("base_link"), cup_link_("cup_link")
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
    cupTransformTimer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestNode::handleCupTransform, this));

    robot_command_sub_ = this->create_subscription<msg_srv::msg::RobotCommand>("robot_command", 10, std::bind(&TestNode::handle_robot_command, this, std::placeholders::_1));
    pickupCupClient_ = this->create_client<msg_srv::srv::PickupCup>("pickup_cup");
}

void TestNode::publish_joint_state()
{
    initTF2();
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

            double angleDestination = ServoUtils::pwmToDegrees(servoCommand.pulseWidth, servoCommand.channel);
            double angleCurrent = Utils::MathUtils::toDegrees(joint_state_message_.position[servoCommand.channel]);

            double angleDifference = angleDestination - angleCurrent;
            double anglePerSecond = servoCommand.speedAnglePerSecond;

            std::cout << "channel: " << servoCommand.channel << std::endl;
            std::cout << "angle difference: " << angleDifference << std::endl;
            std::cout << "angle per second: " << abs(anglePerSecond * 0.01) << std::endl
                      << std::endl;

            // if (servoCommand.channel == 5 || servoCommand.channel == 6)
            // {

            // } else
            if (abs(angleDifference) > abs(anglePerSecond * 0.01) + ServoUtils::getMaxSpeed(servoCommand.channel) * 0.001)
            {
                joint_state_message_.position[servoCommand.channel] += Utils::MathUtils::toRadians(servoCommand.speedAnglePerSecond * 0.01);
                currentCommandFinished = false;
            }
            else
            {
                if (servoCommand.channel == 5 && servoCommand.pulseWidth == 2500)
                {
                    onClosedGripper();
                }
                else if (servoCommand.channel == 5 && servoCommand.pulseWidth == 500)
                {
                    onOpenedGripper();
                }
            }
        }
        if (currentCommandFinished)
        {
            commandQueue.pop();
        }
    }
    publish_joint_state();
}

void TestNode::onClosedGripper()
{
    auto request = std::make_shared<msg_srv::srv::PickupCup::Request>();
    request->pickup = true;

    pickupCupClient_->async_send_request(
        request,
        [this](rclcpp::Client<msg_srv::srv::PickupCup>::SharedFuture result)
        {
            if (result.get()->pickup_success)
            {
                RCLCPP_INFO(this->get_logger(), "Result of pickup: %d", result.get()->pickup_success);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Cup not close enough");
            }
        });
}

void TestNode::onOpenedGripper()
{
    auto request = std::make_shared<msg_srv::srv::PickupCup::Request>();
    request->pickup = false;

    pickupCupClient_->async_send_request(
        request,
        [this](rclcpp::Client<msg_srv::srv::PickupCup>::SharedFuture result)
        {
            if (result.get()->pickup_success)
            {
                RCLCPP_INFO(this->get_logger(), "Result of dropping cup: %d", result.get()->pickup_success);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Cup not close enough");
            }
        });
}

void TestNode::handle_robot_command(const msg_srv::msg::RobotCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->command.c_str());

    if (msg->command == "stop")
    {
        emptyQueue();
        return;
    }

    bool isValid = true;

    CommandParser::CompleteCommand command = parser.parseCompleteCommand(msg->command, isValid);
    parser.calculateRealDuration(command, joint_state_message_);

    if (isValid)
    {
        std::cout << "added command to queue" << std::endl;
        commandQueue.push(command);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Invalid command");
    }
}

void TestNode::emptyQueue()
{
    commandQueue = std::queue<CommandParser::CompleteCommand>();
}

void TestNode::skipCurrentCommand()
{
    if (commandQueue.size() > 0)
    {
        commandQueue.pop();
    }
}

void TestNode::handleCupTransform()
{
    std::string fromFrameRel = sim_link_;
    std::string toFrameRel = cup_link_;

    geometry_msgs::msg::TransformStamped t;

    try
    {
        t = buffer_.lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }
}

void TestNode::initTF2()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = sim_link_;
    t.child_frame_id = bot_link_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    broadcaster_.sendTransform(t);
}