#include "detect_node.h"

#include <chrono>

DetectNode::DetectNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      goal_sent_(false)
{
    action_client_ =
        rclcpp_action::create_client<Detect>(
            node_,
            "/vision/detect_server"
        );
}

BT::PortsList DetectNode::providedPorts()
{
    return {
        BT::InputPort<std::string>("item_group"),
        BT::InputPort<std::string>("item"),
        BT::InputPort<std::string>("camera")
    };
}

BT::NodeStatus DetectNode::onStart()
{
    goal_sent_ = false;
    goal_handle_.reset();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectNode::onRunning()
{
    // ---------------------------------------------------------
    // FASE 1: ESPERANDO O ACTION SERVER E ENVIANDO O GOAL
    // ---------------------------------------------------------

    if (!goal_sent_)
    {
        if (!action_client_->action_server_is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "[%s] Aguardando Detect Action Server...",
                name().c_str()
            );

            return BT::NodeStatus::RUNNING;
        }

        Detect::Goal goal;

        if (!getInput("item_group", goal.item_group))
        {
            throw BT::RuntimeError(
                "Missing required input [item_group]"
            );
        }

        if (!getInput("item", goal.item))
        {
            throw BT::RuntimeError(
                "Missing required input [item]"
            );
        }

        if (!getInput("camera", goal.camera))
        {
            throw BT::RuntimeError(
                "Missing required input [camera]"
            );
        }

        auto send_goal_options =
            rclcpp_action::Client<Detect>::SendGoalOptions();

        goal_handle_future_ =
            action_client_->async_send_goal(
                goal,
                send_goal_options
            );

        goal_sent_ = true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] Solicitando deteccao do item: %s",
            name().c_str(),
            goal.item.c_str()
        );

        return BT::NodeStatus::RUNNING;
    }

    // ---------------------------------------------------------
    // FASE 2: ESPERANDO O GOAL SER ACEITO
    // ---------------------------------------------------------

    if (!goal_handle_)
    {
        if (goal_handle_future_.wait_for(
                std::chrono::milliseconds(0)) !=
            std::future_status::ready)
        {
            return BT::NodeStatus::RUNNING;
        }

        goal_handle_ = goal_handle_future_.get();

        if (!goal_handle_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] Goal de deteccao foi rejeitado.",
                name().c_str()
            );

            return BT::NodeStatus::FAILURE;
        }

        result_future_ =
            action_client_->async_get_result(
                goal_handle_
            );

        return BT::NodeStatus::RUNNING;
    }

    // ---------------------------------------------------------
    // FASE 3: MONITORANDO O RESULTADO
    // ---------------------------------------------------------

    if (result_future_.wait_for(
            std::chrono::milliseconds(0)) !=
        std::future_status::ready)
    {
        return BT::NodeStatus::RUNNING;
    }

    auto result = result_future_.get();

    if (result.code ==
        rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] Deteccao bem sucedida.",
            name().c_str()
        );

        return BT::NodeStatus::SUCCESS;
    }

    if (result.code ==
            rclcpp_action::ResultCode::ABORTED ||
        result.code ==
            rclcpp_action::ResultCode::CANCELED)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] Falha na deteccao.",
            name().c_str()
        );

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void DetectNode::onHalted()
{
    if (action_client_ && goal_handle_)
    {
        action_client_->async_cancel_goal(
            goal_handle_
        );
    }

    goal_handle_.reset();
    goal_sent_ = false;
}
