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

    // split string into commands
    std::vector<std::string> commands = parser.splitCommands(messageString);

    std::cout << "reading commands: " << std::endl;

    for (std::string command : commands)
    {
        std::cout << "Command: '";
        bool isValid = false;

        int channel = parser.getChannel(command, isValid);

        if (isValid)
        {
            std::cout << "Channel: " << channel;
        }
        else
        {
            std::cout << "Channel: invalid";
        }

        int pulseWidth = parser.getPulseWidth(command, isValid);

        if (isValid)
        {
            std::cout << " Pulse Width: " << pulseWidth;
        }
        else
        {
            std::cout << " Pulse Width: invalid";
        }

        int duration = parser.getDuration(command, isValid);

        if (isValid)
        {
            std::cout << " Duration: " << duration;
        }
        else
        {
            std::cout << " Duration: invalid";
        }

        int speed = parser.getSpeed(command, isValid);

        if (isValid)
        {
            std::cout << " Speed: " << speed;
        }
        else
        {
            std::cout << " Speed: invalid";
        }
        std::cout << "'" << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}