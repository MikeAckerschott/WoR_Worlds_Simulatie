#include "cupNode.hpp"

CupNode::CupNode() : Node("cup_node"), buffer(this->get_clock()), listener(buffer), broadcaster(this), simLink("sim_link"), botLink("base_link"), cupLink("cup_link"), isPickedup(false)
{
    initTf2();
    initMarker();
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CupNode::timerCallback, this));

    std::string topic = "/visualization_marker";
    auto qos = rclcpp::QoS(1000);

    markerPub = create_publisher<visualization_msgs::msg::Marker>(topic, qos);
    posPub = create_publisher<msg_srv::msg::Pos>("cup_pos", qos);
    pickupCupService = create_service<msg_srv::srv::PickupCup>("pickup_cup", std::bind(&CupNode::handlePickupCup, this, std::placeholders::_1, std::placeholders::_2));

}

void CupNode::publishMarker()
{
    markerMsg.header.stamp = this->now();

    markerPub->publish(markerMsg);
}

void CupNode::timerCallback()
{

    applyGravity();
    publishMarker();
    broadcastTf2();
    publishCupPos();
}

void CupNode::initMarker()
{
    // Getting the model file path:
    auto package_share_directory = ament_index_cpp::get_package_share_directory("simple_sim_movement");
    std::string base_frame = "base_link";
    auto file_name = "file://" + package_share_directory + "/../../../../wor__worlds_simulatie/simple_sim_movement" + "/stl/cup.stl";

    RCLCPP_INFO(get_logger(), "Waiting for Rviz to load...");

    while (get_node_graph_interface()->count_subscribers("/visualization_marker") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(get_logger(), "Rviz loaded!");

    // Creating the marker and initialising its fields
    // add offset to marker to negate bad model (probably?)
    pose.position.z = -0.01;
    pose.position.x = -0.02;
    pose.position.y = 0.06;

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    std_msgs::msg::ColorRGBA colour;
    colour.a = 1;
    colour.r = 1;
    colour.g = 0;
    colour.b = 0;

    markerMsg.header.frame_id = cupLink;
    markerMsg.header.stamp = now();
    markerMsg.action = visualization_msgs::msg::Marker::ADD;
    markerMsg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    markerMsg.pose = pose;
    markerMsg.id = 0;
    markerMsg.mesh_resource = file_name;
    markerMsg.color = colour;

    markerMsg.scale.x = 0.002;
    markerMsg.scale.y = 0.002;
    markerMsg.scale.z = 0.002;
}

void CupNode::initTf2()
{
    transform.header.frame_id = simLink;
    transform.child_frame_id = cupLink;
    transform.header.stamp = this->now();

    transform.transform.translation.x = 0.4;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    broadcaster.sendTransform(transform);
}

void CupNode::broadcastTf2()
{
    transform.header.stamp = this->now();

    broadcaster.sendTransform(transform);
}

void CupNode::handlePickupCup(const std::shared_ptr<msg_srv::srv::PickupCup::Request> request,
                              const std::shared_ptr<msg_srv::srv::PickupCup::Response> response)
{
    response->pickup_success = false;

    if (request->pickup)
    {
        std::string fromFrameRel = "hand";
        std::string toFrameRel = cupLink;

        geometry_msgs::msg::TransformStamped t;

        try
        {
            t = buffer.lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // check if hand is close enough to cup

        if (t.transform.translation.x < 0.05 && t.transform.translation.y < 0.05 && (t.transform.translation.z < 0.08 || t.transform.translation.z > -0.05))
        {
            handTransformOnPickup = t;

            cupToHand();
            response->pickup_success = true;
        }
    }
    else
    {

        geometry_msgs::msg::TransformStamped newTransform;
        try
        {
            newTransform = buffer.lookupTransform(simLink, cupLink, tf2::TimePointZero);
        }
        catch (tf2::ExtrapolationException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Extrapolation Exception: %s", ex.what());
            // Handle the exception as needed.
        }

        transform.transform = newTransform.transform;
        transform.header.frame_id = simLink;

        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = 0;
        transform.transform.rotation.w = 1;

        isPickedup = false;
    }
}

void CupNode::cupToHand()
{
    std::string targetFrameID = "hand";

    geometry_msgs::msg::TransformStamped newTransform;

    try
    {
        newTransform = buffer.lookupTransform(targetFrameID, cupLink, tf2::TimePointZero);
    }
    catch (tf2::ExtrapolationException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Extrapolation Exception: %s", ex.what());
        // Handle the exception as needed.
    }
    isPickedup = true;

    transform.transform = newTransform.transform;
    transform.header.frame_id = targetFrameID;

    transform.transform.translation.x = handTransformOnPickup.transform.translation.x;
    transform.transform.translation.y = handTransformOnPickup.transform.translation.y;
    transform.transform.translation.z = handTransformOnPickup.transform.translation.z;
}

void CupNode::applyGravity()
{
    if (isPickedup)
    {
        return;
    }

    if (transform.transform.translation.z <= 0.0)
    {
        transform.transform.translation.z = 0.0;
        return;
    }

    // custom gravity to make cup fall slower and more visable
    const float GRAVITY = 0.4;
    const float UPDATES_PER_SECOND = 10;

    // ignore mass and act as if cup is at max speed

    float distance = GRAVITY / UPDATES_PER_SECOND;

    if (transform.transform.translation.z - distance < 0.0)
    {
        transform.transform.translation.z = 0.0;
    }
    else
    {
        transform.transform.translation.z -= distance;
    }
}

void CupNode::publishCupPos(){
    msg_srv::msg::Pos posMsg;

    geometry_msgs::msg::TransformStamped newTransform;

    try
    {
        newTransform = buffer.lookupTransform(simLink, cupLink, tf2::TimePointZero);
    }
    catch (tf2::ExtrapolationException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Extrapolation Exception: %s", ex.what());
        // Handle the exception as needed.
    }

    posMsg.x = newTransform.transform.translation.x;
    posMsg.y = newTransform.transform.translation.y;
    posMsg.z = newTransform.transform.translation.z;

    posPub->publish(posMsg);
}
