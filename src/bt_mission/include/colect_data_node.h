#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>

#include <bt_mission/srv/get_vision_data.hpp>

class ColectDataNode : public BT::SyncActionNode
{
public:

    ColectDataNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<bt_mission::srv::GetVisionData>::SharedPtr client_;
};
