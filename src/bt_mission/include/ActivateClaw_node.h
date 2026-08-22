#ifndef ACTIVATE_CLAW_NODE_H
#define ACTIVATE_CLAW_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ActivateClaw : public BT::SyncActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr claw_pub_;

public:
    ActivateClaw(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("active")
        };
    }
};

#endif
