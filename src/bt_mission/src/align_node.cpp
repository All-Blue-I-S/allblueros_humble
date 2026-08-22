#include "align_node.h"

AlignNode::AlignNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      goal_sent_(false)
{
    action_client_color_ =
        rclcpp_action::create_client<Align>(
            node_,
            "/identifica_cor/align_action_server"
        );

    action_client_yolo_ =
        rclcpp_action::create_client<Align>(
            node_,
            "/yolo_detector/align_action_server"
        );

    cmd_vel_mux_ =
        node_->create_publisher<std_msgs::msg::String>(
            "/mux/cmd_vel",
            rclcpp::QoS(1).transient_local()
        );

    align_mux_ =
        node_->create_publisher<std_msgs::msg::String>(
            "/mux/align",
            rclcpp::QoS(1).transient_local()
        );
}

BT::PortsList AlignNode::providedPorts()
{
    return {
        BT::InputPort<std::string>("item_group"),
        BT::InputPort<std::string>("item"),
        BT::InputPort<std::string>("config"),
        BT::InputPort<std::string>("camera"),
        BT::InputPort<std::string>("tool")
    };
}

BT::NodeStatus AlignNode::onStart()
{
    if (!getInput<std::string>("camera", camera) ||
        !getInput<std::string>("item_group", item_group) ||
        !getInput<std::string>("item", item) ||
        !getInput<std::string>("config", config) ||
        !getInput<std::string>("tool", tool))
    {
        throw BT::RuntimeError(
            "Faltam parâmetros de entrada no XML para AlignNode"
        );
    }

    if (tool != "COLOR" && tool != "YOLO")
    {
        throw BT::RuntimeError(
            "Parametro tool invalido no XML para AlignNode. "
            "Deve ser 'COLOR' ou 'YOLO'."
        );
    }

    // Muda o controle para VS
    std_msgs::msg::String msg;
    msg.data = "VS";
    align_mux_->publish(msg);

    std_msgs::msg::String align_mode;
    align_mode.data = tool;
    align_mux_->publish(align_mode);

    goal_sent_ = false;
    goal_handle_.reset();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignNode::onRunning()
{
    // ---------------------------------------------------------
    // FASE 1: ESPERANDO O SERVIDOR E ENVIANDO O OBJETIVO
    // ---------------------------------------------------------

    if (!goal_sent_)
    {
        // Seleciona o Action Client de acordo com a ferramenta
        if (tool == "COLOR")
        {
            action_client_ = action_client_color_;

            if (!action_client_->action_server_is_ready())
            {
                return BT::NodeStatus::RUNNING;
            }
        }
        else if (tool == "YOLO")
        {
            action_client_ = action_client_yolo_;

            if (!action_client_->action_server_is_ready())
            {
                return BT::NodeStatus::RUNNING;
            }
        }
        else
        {
            throw BT::RuntimeError(
                "Parametro tool invalido no XML para AlignNode"
            );
        }

        // -----------------------------------------------------
        // Configura o Goal
        // -----------------------------------------------------

        Align::Goal goal;

        goal.cam = camera;
        goal.item_group = item_group;
        goal.item = item;
        goal.config = config;

        // -----------------------------------------------------
        // Envia o Goal
        // -----------------------------------------------------

        auto send_goal_options =
            rclcpp_action::Client<Align>::SendGoalOptions();

        auto future_goal_handle =
            action_client_->async_send_goal(
                goal,
                send_goal_options
            );

        // Veremos abaixo como tratar esse Future.
        // Por enquanto armazenamos o estado da operação.
        goal_sent_ = true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] Enviado comando de tracking para o item %s "
            "na camera: %s",
            name().c_str(),
            item.c_str(),
            camera.c_str()
        );

        return BT::NodeStatus::RUNNING;
    }

    // ---------------------------------------------------------
    // FASE 2: MONITORANDO O PROGRESSO
    // ---------------------------------------------------------

    if (!goal_handle_)
    {
        return BT::NodeStatus::RUNNING;
    }

    auto result_future = action_client_->async_get_result(
        goal_handle_
    );

    if (result_future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
    {
        return BT::NodeStatus::RUNNING;
    }

    auto wrapped_result = result_future.get();

    if (wrapped_result.code ==
        rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] Erro visual minimizado. "
            "Rastreamento concluido com sucesso.",
            name().c_str()
        );

        std_msgs::msg::String msg;
        msg.data = "IDLE";

        cmd_vel_mux_->publish(msg);
        align_mux_->publish(msg);

        return BT::NodeStatus::SUCCESS;
    }

    if (wrapped_result.code ==
            rclcpp_action::ResultCode::ABORTED ||
        wrapped_result.code ==
            rclcpp_action::ResultCode::CANCELED)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] Falha no rastreamento visual.",
            name().c_str()
        );

        std_msgs::msg::String msg;
        msg.data = "IDLE";

        cmd_vel_mux_->publish(msg);
        align_mux_->publish(msg);

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void AlignNode::onHalted()
{
    // Volta o controle para IDLE
    std_msgs::msg::String msg;
    msg.data = "IDLE";

    cmd_vel_mux_->publish(msg);
    align_mux_->publish(msg);

    RCLCPP_WARN(
        node_->get_logger(),
        "[%s] Cancelado pela BT! Parando rastreamento.",
        name().c_str()
    );

    // Cancela o objetivo atual
    if (action_client_ && goal_handle_)
    {
        action_client_->async_cancel_goal(goal_handle_);
    }

    goal_handle_.reset();
    goal_sent_ = false;
}
