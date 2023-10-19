#include "cupNode.hpp"

CupNode::CupNode() : Node("cup_node"), buffer_(this->get_clock()), listener_(buffer_), broadcaster_(this), sim_link_("sim_link"), bot_link_("base_link"), cup_link_("cup_link")
{
    initTF2();
    initMarker();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CupNode::timerCallback, this));

    std::string topic = "/visualization_marker";
    auto qos = rclcpp::QoS(1000);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(topic, qos);
}

void CupNode::publishMarker()
{
    marker_msg.header.stamp = this->now();
    RCLCPP_INFO(get_logger(), "Attempting to publish mesh");

    marker_pub_->publish(marker_msg);
}

void CupNode::timerCallback()
{
    std::cout << "publishing marker" << std::endl;
    publishMarker();
    //TODO REMOVE
    initTF2();
    broadcaster_.sendTransform(transform_);
}

void CupNode::initMarker()
{
    // Getting the model file path:
    auto package_share_directory = ament_index_cpp::get_package_share_directory("simple_sim_movement");
    std::string base_frame = "base_link";
    auto file_name = "file://" + package_share_directory + "/../../../../simple_sim_movement" + "/stl/cup.stl";

    RCLCPP_INFO(get_logger(), "Waiting for Rviz to load...");

    while (get_node_graph_interface()->count_subscribers("/visualization_marker") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // Creating the marker and initialising its fields
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.2;
    pose.position.y = 0.1;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    std_msgs::msg::ColorRGBA colour;
    colour.a = 1;
    colour.r = 1;
    colour.g = 0;
    colour.b = 0;

    marker_msg.header.frame_id = "base_link";
    marker_msg.header.stamp = now();
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_msg.pose = pose;
    marker_msg.id = 0;
    marker_msg.mesh_resource = file_name;
    marker_msg.color = colour;

    marker_msg.scale.x = 0.003;
    marker_msg.scale.y = 0.003;
    marker_msg.scale.z = 0.003;
}

void CupNode::initTF2()
{
    transform_.header.frame_id = sim_link_;
    transform_.child_frame_id = cup_link_;
    transform_.header.stamp = this->now();

    transform_.transform.translation.x = 0.17;
    transform_.transform.translation.y = 0.19;
    transform_.transform.translation.z = 0.0;

    broadcaster_.sendTransform(transform_);
}
