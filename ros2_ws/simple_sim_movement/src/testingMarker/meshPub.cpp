#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);

    rclcpp::Node node("shape_pubilsher");

    string base_frame = "base_link";

    string topic = "/visualization_marker";

    // Getting the model file path:
    auto package_share_directory = ament_index_cpp::get_package_share_directory("simple_sim_movement");

    auto file_name = "file://" + package_share_directory + "/../../../../simple_sim_movement" + "/stl/cup.stl";

    auto qos = rclcpp::QoS(1000);

    auto publisher = node.create_publisher<visualization_msgs::msg::Marker>(topic, qos);

    RCLCPP_INFO(node.get_logger(), "Waiting for Rviz to load...");

    while (node.get_node_graph_interface()->count_subscribers(topic) == 0)
    {
        rclcpp::sleep_for(200ms);
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

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = base_frame;
    marker.header.stamp = node.now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.pose = pose;
    marker.id = 0;
    marker.mesh_resource = file_name;
    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;
    marker.color = colour;

    marker.scale.x = 0.003;
    marker.scale.y = 0.003;
    marker.scale.z = 0.003;

    RCLCPP_INFO(node.get_logger(), "Attempting to publish mesh");

    publisher->publish(marker);

    while (rclcpp::ok())
        ;

    return 0;
}