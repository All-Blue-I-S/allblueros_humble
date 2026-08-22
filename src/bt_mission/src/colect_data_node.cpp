#include "colect_data_node.h"

#include <chrono>

ColectDataNode::ColectDataNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    client_ = node_->create_client<bt_mission::srv::GetVisionData>(
        "/vision/get_data"
    );
}

BT::PortsList ColectDataNode::providedPorts()
{
    return {
        BT::OutputPort<std::string>("image_grup"),
        BT::OutputPort<std::string>("side"),
        BT::OutputPort<std::string>("other_image")
    };
}

BT::NodeStatus ColectDataNode::tick()
{
    auto request =
        std::make_shared<
            bt_mission::srv::GetVisionData::Request
        >();

    // Verifica se o serviço está disponível
    if (!client_->wait_for_service(std::chrono::milliseconds(0)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "ColectDataNode: Falha ao contatar o servico de visao."
        );

        return BT::NodeStatus::FAILURE;
    }

    // Envia a requisição
    auto future = client_->async_send_request(request);

    // Aguarda a resposta
    if (rclcpp::spin_until_future_complete(
            node_,
            future
        ) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "ColectDataNode: Falha ao contatar o servico de visao."
        );

        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();

    // Salva os retornos do YOLO no Blackboard
    setOutput(
        "image_grup",
        response->image_group
    );

    setOutput(
        "side",
        response->side
    );

    setOutput(
        "other_image",
        response->other_image
    );

    RCLCPP_INFO(
        node_->get_logger(),
        "ColectDataNode: Dados coletados com sucesso."
    );

    return BT::NodeStatus::SUCCESS;
}
