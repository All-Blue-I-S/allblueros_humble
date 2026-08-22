#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel.hpp>

class EmergeNode : public BT::StatefulActionNode
{
public:

    EmergeNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr cmd_accel_pub_;

    rclcpp::Time start_time_;
};
