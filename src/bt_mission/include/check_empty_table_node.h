#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

#include <rclcpp/rclcpp.hpp>

#include <bt_mission/srv/check_empty.hpp>

class CheckEmptyTableNode : public BT::ConditionNode
{
public:
    CheckEmptyTableNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<bt_mission::srv::CheckEmpty>::SharedPtr client_;
};
