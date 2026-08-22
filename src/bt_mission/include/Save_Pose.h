#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SavePose : public BT::SyncActionNode
{
public:
    SavePose(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    nav_msgs::msg::Odometry current_odom_;

    bool odom_received_;

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);
};
