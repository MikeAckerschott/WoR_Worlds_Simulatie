// Purpose: main file for testing the communication between the testNode and the mainCommunicatorNode

#include <iostream>
#include "communicatorNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    bool exitRequested = false;

    std::cout<<"Welcome to the simulated AL5D robotic arm user interface!"<<std::endl;
    std::cout<<"For command format, please refer to the docs"<<std::endl;

    std::shared_ptr<CommunicatorNode> node =
        std::make_shared<CommunicatorNode>();

    while (rclcpp::ok() && !exitRequested)
    {
        std::cout << "Enter command: " << std::endl;
        // Check for user input
        std::string input;
        std::getline(std::cin, input);

        if (input == "exit")
        {
            exitRequested = true;
            break;
        }

        node->sendCommand(input);
    }
    rclcpp::shutdown();
    return 0;
}