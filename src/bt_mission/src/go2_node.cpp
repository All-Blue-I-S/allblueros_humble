#include "go2_node.h"

#include <chrono>

Go2Node::Go2Node(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      goal_sent_(false)
{
    action_client_ =
        rclcpp_action::create_client<Go2>(
            node_,
            "/go2_server"
        );

    mux_pub_ =
        node_->create_publisher<std_msgs::msg::String>(
            "/mux/cmd_vel",
            rclcpp::QoS(1).transient_local()
        );
}

BT::PortsList Go2Node::providedPorts()
{
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<std::string>("frame")
    };
}

BT::NodeStatus Go2Node::onStart()
{
    goal_sent_ = false;
    goal_handle_.reset();

    // Verifica instantaneamente se o servidor está disponível.
    // Se não estiver, o onRunning continuará verificando.
    if (!action_client_->action_server_is_ready())
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[Go2] Action Server não está disponível."
        );

        return BT::NodeStatus::RUNNING;
    }

    double x;
    double y;
    double z;
    std::string frame_id;

    if (!getInput<double>("x", x) ||
        !getInput<double>("y", y) ||
        !getInput<double>("z", z) ||
        !getInput<std::string>("frame", frame_id))
    {
        throw BT::RuntimeError(
            "[Go2] Faltam parâmetros de posição no XML"
        );
    }

    // ---------------------------------------------------------
    // Publica no mux para garantir que o modo de controle
    // está correto
    // ---------------------------------------------------------

    std_msgs::msg::String msg;
    msg.data = "GO2";

    mux_pub_->publish(msg);

    // ---------------------------------------------------------
    // Preenche o Goal
    // ---------------------------------------------------------

    Go2::Goal goal;

    goal.target_pose.header.frame_id = frame_id;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;

    goal.target_pose.pose.orientation.w = 1.0;

    // ---------------------------------------------------------
    // Envia o Goal
    // ---------------------------------------------------------

    goal_handle_future_ =
        action_client_->async_send_goal(goal);

    goal_sent_ = true;

    RCLCPP_INFO(
        node_->get_logger(),
        "Enviado objetivo: [%.1f, %.1f, %.1f] frame: %s",
        x,
        y,
        z,
        frame_id.c_str()
    );

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Go2Node::onRunning()
{
    // ---------------------------------------------------------
    // Espera o Goal ser aceito pelo Action Server
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
                "[Go2] Goal rejeitado pelo Action Server."
            );

            return BT::NodeStatus::FAILURE;
        }

        // Agora esperamos o resultado da Action
        result_future_ =
            action_client_->async_get_result(
                goal_handle_
            );

        return BT::NodeStatus::RUNNING;
    }

    // ---------------------------------------------------------
    // Monitora o resultado
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
            "Go2 finalizado com sucesso."
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
            "Go2 falhou no servidor."
        );

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void Go2Node::onHalted()
{
    // Coloca o mux em IDLE
    std_msgs::msg::String msg;
    msg.data = "IDLE";

    mux_pub_->publish(msg);

    RCLCPP_WARN(
        node_->get_logger(),
        "Go2 Abortado pela Behavior Tree! Cancelando objetivo."
    );

    if (goal_handle_)
    {
        action_client_->async_cancel_goal(
            goal_handle_
        );
    }

    goal_handle_.reset();
    goal_sent_ = false;
}
