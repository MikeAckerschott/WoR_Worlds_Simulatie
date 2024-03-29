#include "CupStatePublisher.hpp"
#include <chrono>
#include <fstream>
#include <thread>

CupStatePublisher::CupStatePublisher() : Node("cup_state_publisher"), speedFactor(1) {
    redCup = this->declare_parameter("red_cup", std::string(""));

    currentTransform.header.frame_id = "base_link";
    currentTransform.child_frame_id = "cup";
    currentTransform.transform.translation.x = 0.45;
    currentTransform.transform.translation.y = 0.0;
    currentTransform.transform.translation.z = 0.05;
    currentTransform.transform.rotation.x = 0.0;
    currentTransform.transform.rotation.y = 0.0;
    currentTransform.transform.rotation.z = 0.0;

    description_pub_ = this->create_publisher<std_msgs::msg::String>(
    "cup_description",
    // Transient local is similar to latching in ROS 1.
    rclcpp::QoS(1).transient_local());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    setupUrdf(redCup);

    //log
    RCLCPP_INFO(this->get_logger(), "URDF SENT: %s", redCup.c_str());
    
    this->timer = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        this->publishCup();
    });
}

void CupStatePublisher::updatePos(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
}

void CupStatePublisher::setupUrdf(std::string urdf) {
    auto msg = std::make_unique<std_msgs::msg::String>();

    msg->data = urdf;

    description_pub_->publish(std::move(msg));
}

void CupStatePublisher::publishCup() {
    geometry_msgs::msg::TransformStamped t;

    t = currentTransform;
    t.header.stamp = this->now();

    tf_broadcaster_->sendTransform(t);
}

CupStatePublisher::~CupStatePublisher() {

}