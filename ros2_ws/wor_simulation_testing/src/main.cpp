#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "testnode.hpp"
#include "commandParser.hpp"

// include cin
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto parser = CommandParser();

    // wait for string in cin
    std::string messageString;
    std::getline(std::cin, messageString);

    bool isValid = false;

    auto fullCommand = parser.parseCompleteCommand(messageString, isValid);

    if (isValid)
    {

        std::cout << "parsed command: " << std::endl;
        if(fullCommand.usingDuration)
        {
            std::cout << "duration: " << fullCommand.duration << std::endl;
        }

        for (auto servoCommand : fullCommand.servoCommands)
        {
            std::cout << "channel: " << servoCommand.channel << std::endl;
            std::cout << "pulse width: " << servoCommand.pulseWidth << std::endl;
            if(servoCommand.usingSpeed)
            {
                std::cout << "speed: " << servoCommand.speed << std::endl;
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}