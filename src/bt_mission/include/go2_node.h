#ifndef GO2_NODE_H
#define GO2_NODE_H

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>

#include <bt_mission/action/go2.hpp>

#include <future>

class Go2Node : public BT::StatefulActionNode
{
private:
    using Go2 = bt_mission::action::Go2;
    using GoalHandleGo2 = rclcpp_action::ClientGoalHandle<Go2>;

    rclcpp::Node::SharedPtr node_;

    rclcpp_action::Client<Go2>::SharedPtr action_client_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mux_pub_;

    std::shared_future<GoalHandleGo2::SharedPtr> goal_handle_future_;
    std::shared_future<GoalHandleGo2::WrappedResult> result_future_;

    GoalHandleGo2::SharedPtr goal_handle_;

    bool goal_sent_;

public:
    Go2Node(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

#endif // GO2_NODE_H
