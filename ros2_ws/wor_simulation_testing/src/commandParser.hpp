#ifndef COMMANDPARSER_HPP
#define COMMANDPARSER_HPP

#include <string>
#include <vector>
#include <iostream>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "servoUtils.hpp"
#include "mathUtils.hpp"

class CommandParser
{
public:
    CommandParser();
    virtual ~CommandParser() = default;

    struct ServoCommand
    {
        int channel;
        int pulseWidth;
        int speedPWM;
        bool usingSpeed;

        double speedAnglePerSecond;
    };

    struct CompleteCommand
    {
        std::vector<ServoCommand> servoCommands;
        int duration;
        bool usingDuration;
        unsigned long long startTime;
    };

    CommandParser::CompleteCommand parseCompleteCommand(std::string &messageString, bool &isValid);
    int calculateRealDuration(CompleteCommand &command, sensor_msgs::msg::JointState robotPositionMessage_);

private:
    int parseIntAfterChar(char delimiter, bool &isValid, std::string &messageString);
    int getChannel(std::string &messageString, bool &isChannel);
    int getPulseWidth(std::string &messageString, bool &isChannel);
    int getDuration(std::string &messageString, bool &isChannel);
    int getSpeed(std::string &messageString, bool &isChannel);

    std::vector<std::string> splitCommands(std::string &messageString);

    CommandParser::ServoCommand parseServoCommand(std::string &messageString, bool &isValid, bool &usingDuration);


};

#endif /* COMMANDPARSER_HPP */
