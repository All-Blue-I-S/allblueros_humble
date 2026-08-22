#include "check_empty_table_node.h"

#include <chrono>

CheckEmptyTableNode::CheckEmptyTableNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config),
      node_(node)
{
    client_ = node_->create_client<bt_mission::srv::CheckEmpty>(
        "/vision/check_empty"
    );
}

BT::NodeStatus CheckEmptyTableNode::tick()
{
    auto request =
        std::make_shared<bt_mission::srv::CheckEmpty::Request>();

    request->target_area = "table";

    // Verifica se o serviço está disponível
    if (!client_->wait_for_service(std::chrono::milliseconds(0)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "CheckEmptyTable: Servico de visao indisponivel."
        );

        return BT::NodeStatus::FAILURE;
    }

    // Envia a requisição
    auto future = client_->async_send_request(request);

    // Espera pela resposta
    if (rclcpp::spin_until_future_complete(
            node_,
            future
        ) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "CheckEmptyTable: Falha ao chamar servico de visao."
        );

        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();

    if (response->is_empty)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "CheckEmptyTable: A mesa esta vazia."
        );

        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "CheckEmptyTable: Ainda existem itens na mesa."
    );

    return BT::NodeStatus::FAILURE;
}
