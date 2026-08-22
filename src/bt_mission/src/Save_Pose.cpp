#include "Save_Pose.h"

#include <functional>

SavePose::SavePose(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node),
      odom_received_(false)
{
    odom_sub_ =
        node_->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom",
            10,
            std::bind(
                &SavePose::odomCallback,
                this,
                std::placeholders::_1));
}


BT::PortsList SavePose::providedPorts()
{
    return {
        BT::OutputPort<double>("x"),
        BT::OutputPort<double>("y"),
        BT::OutputPort<double>("z")
    };
}


void SavePose::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;
    odom_received_ = true;
}


BT::NodeStatus SavePose::tick()
{
    if (!odom_received_)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "SavePose: odometria ainda não recebida.");

        return BT::NodeStatus::FAILURE;
    }


    setOutput(
        "x",
        current_odom_.pose.pose.position.x);

    setOutput(
        "y",
        current_odom_.pose.pose.position.y);

    setOutput(
        "z",
        current_odom_.pose.pose.position.z);


    RCLCPP_INFO(
        node_->get_logger(),
        "Posição salva: %.2f %.2f %.2f",
        current_odom_.pose.pose.position.x,
        current_odom_.pose.pose.position.y,
        current_odom_.pose.pose.position.z);


    return BT::NodeStatus::SUCCESS;
}
