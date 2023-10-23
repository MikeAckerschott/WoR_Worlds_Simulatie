#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "msg_srv/srv/pickup_cup.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "../utils/mathUtils.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class CupNode : public rclcpp::Node
{
public:
    CupNode();

    virtual ~CupNode() = default;

    void publishMarker();

    void timerCallback();

private:
    void initMarker();
    void initTf2();
    void markerToTf2();

    void handlePickupCup(const std::shared_ptr<msg_srv::srv::PickupCup::Request> request, const std::shared_ptr<msg_srv::srv::PickupCup::Response> response);
    void broadcastTf2();

    void cupToHand();

private:
    const float MARKER_TF2_X_OFFSET = 0.03;
    const float MARKER_TF2_Y_OFFSET = 0.01;

    std::string simLink, botLink, cupLink;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    rclcpp::Service<msg_srv::srv::PickupCup>::SharedPtr pickupCupService;
    visualization_msgs::msg::Marker markerMsg;
    geometry_msgs::msg::Pose pose;

    rclcpp::TimerBase::SharedPtr timer;

    geometry_msgs::msg::TransformStamped transform;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;

    bool isPickedup = true;
};
