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
}

void TestNode::publishJointState()
{
    jointStateMessage_.header.stamp = now();     // set time
                                                 // fill msg
    jointStatePub_->publish(jointStateMessage_); // send
}

void TestNode::handleSubRobotCommand(const msg_srv::msg::RobotCommand::SharedPtr msg)
{
    std::string messageString = msg->command;

    /**
     * PARSE COMMAND. SEE Lynxmotion SSC-32U USB Servo Controller Board User Manual
     *
     * "#<ch>P<pw>​S​<spd>​T<time><cr>"
     * <ch>   : pin / channel to which the servo is connected (0 to 31) in decimal
     * <pw>   : desired pulse width (normally 500 to 2500) in microseconds
     * <spd>​  : servo movement speed in microseconds per second*
     * <time>​ : time in microseconds to travel from the current position to the desired position. This affects all servos (65535 max) *
     */

    // get first int after #
    int iterator = 0;

    // check if selected channel is actually a number
    if (messageString[0] != '#')
    {
        std::cout << "ERROR: Command does not start with #. Command: " << messageString << std::endl;
        return;
    }

    std::string chString = "";

    while (messageString[iterator] != '\0')
    {
        if (messageString[iterator] == ' ')
        {
            iterator++;
            continue;
        }
        if (messageString[iterator] < '0' || messageString[iterator] > '9')
        {
            break;
        }
        chString += messageString[iterator];
        iterator++;
    }

    if(messageString[iterator] == '\0' || chString.size() == 0){
        std::cout << "ERROR: Command does not contain a channel. Command: " << messageString << std::endl;
        return;
    }

    // get integer value from char found at iterator
    int channel = messageString[iterator] - '0';

    //Find P
    int pIndex = messageString.find('P');
    iterator = pIndex + 1;
    std::string pwString = "";

    // loop through input string untill a non-numeric value (ignoring spaces) is found or end of string is reached
    while (messageString[iterator] != '\0')
    {
        if (messageString[iterator] == ' ')
        {
            iterator++;
            continue;
        }
        if (messageString[iterator] < '0' || messageString[iterator] > '9')
        {
            break;
        }
        pwString += messageString[iterator];
        iterator++;
    }

    if(messageString[iterator] == '\0'){
        std::cout << "ERROR: Command does not contain a pulse width. Command: " << messageString << std::endl;
        return;
    }

    // get integer value from char found at iterator
    int pulseWidth = std::stoi(pwString);

}