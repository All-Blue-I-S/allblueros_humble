#include "DropMarkers_node.h"

#include <chrono>
#include <thread>

DropMarkers::DropMarkers(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    marker_pub_ =
        node_->create_publisher<std_msgs::msg::UInt32>(
            "marker_topic",
            10
        );
}

BT::NodeStatus DropMarkers::tick()
{
    unsigned int dropper;

    if (!getInput<unsigned int>("dropper", dropper))
    {
        throw BT::RuntimeError(
            "missing required input [dropper]"
        );
    }

    std_msgs::msg::UInt32 msg;
    msg.data = dropper;

    marker_pub_->publish(msg);

    // Simula o tempo necessário para soltar os marcadores
    std::this_thread::sleep_for(
        std::chrono::seconds(1)
    );

    return BT::NodeStatus::SUCCESS;
}
