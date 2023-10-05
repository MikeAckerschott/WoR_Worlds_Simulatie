#include "testnode.hpp"

TestNode::TestNode() : Node("test_node")
{
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_state_message_.name = {
            "base_link2turret",
            "turret2upperarm",
            "upperarm2forearm",
            "forearm2wrist",
            "wrist2hand",
            "gripper_left2hand",
            "gripper_right2hand"};
        joint_state_message_.position = {
            0, // base
            0, // shoulder
            0, // elbow
            0, // wrist
            0, // hand
            0, // gripper 01
            0, // gripper 02
        };
        joint_state_pub_->publish(joint_state_message_);
}

void TestNode::publish_joint_state()
{
        joint_state_message_.header.stamp = now();              //set time
                                                                //fill msg
        joint_state_pub_->publish(joint_state_message_);        //send
}