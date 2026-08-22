#ifndef ALIGN_NODE_H
#define ALIGN_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>
#include <bt_mission/action/align.hpp>

class AlignNode : public BT::StatefulActionNode
{
    /**
     * Classe que representa um nó de ação para o movimento "Align"
     * em uma árvore de comportamento.
     *
     * Esse nó altera o modo de controle para VS, usando o
     * Visual Servoing para controlar o movimento.
     */

private:
    using Align = bt_mission::action::Align;
    using GoalHandleAlign = rclcpp_action::ClientGoalHandle<Align>;

    rclcpp::Node::SharedPtr node_;

    rclcpp_action::Client<Align>::SharedPtr action_client_color_;
    rclcpp_action::Client<Align>::SharedPtr action_client_yolo_;
    rclcpp_action::Client<Align>::SharedPtr action_client_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr align_mux_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_vel_mux_;

    GoalHandleAlign::SharedPtr goal_handle_;

    bool goal_sent_;

    std::string camera;
    std::string item_group;
    std::string item;
    std::string config;
    std::string tool;

public:
    AlignNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

#endif // ALIGN_NODE_H
