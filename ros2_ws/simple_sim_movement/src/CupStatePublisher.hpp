#ifndef CUPSTATEPUBLISHER_HPP__
#define CUPSTATEPUBLISHER_HPP__

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <map>
#include <std_msgs/msg/string.hpp>

class CupStatePublisher : public rclcpp::Node
{
public:
    CupStatePublisher();
    void setupUrdf(std::string urdf);
    void publishCup();
    ~CupStatePublisher();
private:
    void updatePos(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer;

    /// A pointer to the ROS 2 publisher for the robot_description
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cupPickedUpSub;
    std::string redCup;
    geometry_msgs::msg::TransformStamped currentTransform;
    uint8_t speedFactor;
};


#endif // CUPSTATEPUBLISHER_HPP__