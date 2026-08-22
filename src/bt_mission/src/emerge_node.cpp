#include "emerge_node.h"

EmergeNode::EmergeNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node)
{
    cmd_accel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Accel>(
            "/cmd_accel",
            1
        );
}

BT::NodeStatus EmergeNode::onStart()
{
    start_time_ = node_->now();

    RCLCPP_INFO(
        node_->get_logger(),
        "EmergeNode: Iniciando subida para a superficie."
    );

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EmergeNode::onRunning()
{
    // Verifica se já passaram 5 segundos
    if ((node_->now() - start_time_).seconds() < 5.0)
    {
        geometry_msgs::msg::Accel cmd;

        cmd.linear.z = 1.0;

        cmd_accel_pub_->publish(cmd);

        return BT::NodeStatus::RUNNING;
    }

    // Passou de 5 segundos, para o AUV
    geometry_msgs::msg::Accel stop_cmd;

    stop_cmd.linear.z = 0.0;

    cmd_accel_pub_->publish(stop_cmd);

    RCLCPP_INFO(
        node_->get_logger(),
        "EmergeNode: 5 segundos concluidos."
    );

    return BT::NodeStatus::SUCCESS;
}

void EmergeNode::onHalted()
{
    geometry_msgs::msg::Accel stop_cmd;

    stop_cmd.linear.z = 0.0;

    cmd_accel_pub_->publish(stop_cmd);

    RCLCPP_WARN(
        node_->get_logger(),
        "EmergeNode: Abortado pela BT!"
    );
}
