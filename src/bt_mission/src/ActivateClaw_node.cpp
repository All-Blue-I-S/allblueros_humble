#include "ActivateClaw_node.h"

#include <chrono>

ActivateClaw::ActivateClaw(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    claw_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "claw_topic",
        10
    );
}

BT::NodeStatus ActivateClaw::tick()
{
    bool active;

    if (!getInput<bool>("active", active))
    {
        throw BT::RuntimeError("Missing required input [active]");
    }

    std_msgs::msg::String msg;
    msg.data = active ? "Activate claw!" : "Deactivate claw!";

    claw_pub_->publish(msg);

    rclcpp::sleep_for(std::chrono::seconds(1));

    return BT::NodeStatus::SUCCESS;
}
