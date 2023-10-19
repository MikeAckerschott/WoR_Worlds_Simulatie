#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <chrono>

class CupNode : public rclcpp::Node
{
public:
    CupNode();

    virtual ~CupNode() = default;

    void publishMarker();

    void timerCallback();

private:
    void initMarker();
    void initTF2();

private:
    std::string sim_link_, bot_link_, cup_link_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker marker_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped transform_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
};
