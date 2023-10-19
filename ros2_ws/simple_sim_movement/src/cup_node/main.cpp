#include <iostream>
#include "cupNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CupNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
