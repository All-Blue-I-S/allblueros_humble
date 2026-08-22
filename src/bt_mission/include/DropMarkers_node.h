#ifndef DROP_MARKERS_NODE_H
#define DROP_MARKERS_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int32.hpp>

class DropMarkers : public BT::SyncActionNode
{
private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr marker_pub_;

public:
    DropMarkers(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<unsigned int>(
                "dropper",
                "The dropper that should be activated"
            )
        };
    }
};

#endif  // DROP_MARKERS_NODE_H
