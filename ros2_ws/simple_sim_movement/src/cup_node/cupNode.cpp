#include "cupNode.hpp"

CupNode::CupNode() : Node("cup_node"), buffer_(this->get_clock()), listener_(buffer_), broadcaster_(this), sim_link_("sim_link"), bot_link_("base_link"), cup_link_("cup_link")
{
    initTF2();
    initMarker();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CupNode::timerCallback, this));


}

void CupNode::publishMarker()
{
    marker_msg.header.stamp = this->now();
    marker_pub_->publish(std::move(marker_msg));
}

void CupNode::timerCallback()
{
    std::cout << "publishing marker" << std::endl;
    publishMarker();
    broadcaster_.sendTransform(transform_);
}

void CupNode::initMarker()
{
    marker_msg = visualization_msgs::msg::Marker();

    marker_msg.header.frame_id = cup_link_;
    marker_msg.header.stamp = this->now();

    marker_msg.ns = "wor_cup";
    marker_msg.id = 0;

    marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;

    marker_msg.pose.position.x = 1.0;
    marker_msg.pose.position.y = 2.0;
    marker_msg.pose.position.z = 0.0;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("simple_sim_movement");
    std::string file_name = package_share_directory.append("/stl/cup.stl");

    marker_msg.mesh_resource = "file://" + file_name;

    // marker_msg.scale.x = 0.001;
    // marker_msg.scale.y = 0.001;
    // marker_msg.scale.z = 0.001;

    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;

    marker_msg.scale.x = 1.0;
    marker_msg.scale.y = 1.0;
    marker_msg.scale.z = 1.0;

    marker_msg.color.r = 0.0f;
    marker_msg.color.g = 1.0f;
    marker_msg.color.b = 0.0f;
    marker_msg.color.a = 1.0;

    marker_msg.lifetime = rclcpp::Duration(std::chrono::milliseconds(10));
}

void CupNode::initTF2() {
    transform_.header.frame_id = sim_link_;
    transform_.child_frame_id = cup_link_;
    transform_.header.stamp = this->now();

    //TODO add functionality to get object position from rviz
    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;

    broadcaster_.sendTransform(transform_);
}
