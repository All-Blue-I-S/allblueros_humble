#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <bt_mission/action/detect.hpp>

#include <future>

class DetectNode : public BT::StatefulActionNode
{
public:

    DetectNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:

    using Detect = bt_mission::action::Detect;
    using GoalHandleDetect = rclcpp_action::ClientGoalHandle<Detect>;

    rclcpp::Node::SharedPtr node_;

    rclcpp_action::Client<Detect>::SharedPtr action_client_;

    std::shared_future<GoalHandleDetect::SharedPtr> goal_handle_future_;

    std::shared_future<GoalHandleDetect::WrappedResult> result_future_;

    GoalHandleDetect::SharedPtr goal_handle_;

    bool goal_sent_;
};
