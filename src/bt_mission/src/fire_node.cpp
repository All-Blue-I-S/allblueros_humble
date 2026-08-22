#include "fire_node.h"

#include <chrono>
#include <thread>

FireNode::FireNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::AsyncActionNode(name, config),
      node_(node),
      halt_requested_(false)
{
    mavros_client_ =
        node_->create_client<mavros_msgs::srv::CommandLong>(
            "/mavros/cmd/command"
        );
}

BT::NodeStatus FireNode::tick()
{
    halt_requested_ = false;

    unsigned int relay_id;
    int cycles;
    float cycle_time;

    // Busca os parâmetros definidos no XML
    if (!getInput<unsigned int>("relay_id", relay_id) ||
        !getInput<int>("cycles", cycles) ||
        !getInput<float>("cycle_time", cycle_time))
    {
        throw BT::RuntimeError(
            "Faltam parâmetros obrigatorios "
            "(relay_id, cycles, cycle_time) no XML"
        );
    }

    // ---------------------------------------------------------
    // Prepara o comando MAV_CMD_DO_REPEAT_RELAY
    // ---------------------------------------------------------

    auto request =
        std::make_shared<
            mavros_msgs::srv::CommandLong::Request
        >();

    request->broadcast = false;
    request->command = 182; // MAV_CMD_DO_REPEAT_RELAY
    request->confirmation = 0;

    request->param1 = relay_id;
    request->param2 = cycles;
    request->param3 = cycle_time;

    // Verifica se o serviço está disponível
    if (!mavros_client_->wait_for_service(
            std::chrono::milliseconds(0)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Falha ao comunicar com MAVROS para metralhar o rele %u",
            relay_id
        );

        return BT::NodeStatus::FAILURE;
    }

    // Envia o comando
    auto future =
        mavros_client_->async_send_request(request);

    // Espera a resposta do serviço
    if (rclcpp::spin_until_future_complete(
            node_,
            future
        ) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Falha ao comunicar com MAVROS para metralhar o rele %u",
            relay_id
        );

        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();

    if (!response->success)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "MAVROS rejeitou o comando para o rele %u",
            relay_id
        );

        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Modo metralhadora! Rele %u disparando %d vezes "
        "a cada %.2f segundos.",
        relay_id,
        cycles,
        cycle_time
    );

    // ---------------------------------------------------------
    // Espera pelo tempo total da operação
    // ---------------------------------------------------------

    const auto total_duration =
        std::chrono::duration<float>(
            cycles * cycle_time
        );

    const auto check_interval =
        std::chrono::milliseconds(100);

    auto elapsed = std::chrono::milliseconds(0);

    while (
        elapsed <
        std::chrono::duration_cast<std::chrono::milliseconds>(
            total_duration
        )
    )
    {
        // Verifica se a BT solicitou abortar
        if (halt_requested_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Comando de metralhadora abortado pela BT!"
            );

            // Força o relé para LOW usando
            // MAV_CMD_DO_SET_RELAY (181)
            auto stop_request =
                std::make_shared<
                    mavros_msgs::srv::CommandLong::Request
                >();

            stop_request->broadcast = false;
            stop_request->command = 181; // MAV_CMD_DO_SET_RELAY
            stop_request->param1 = relay_id;
            stop_request->param2 = 0.0;

            // Não precisamos bloquear esperando a resposta.
            mavros_client_->async_send_request(
                stop_request
            );

            return BT::NodeStatus::IDLE;
        }

        std::this_thread::sleep_for(check_interval);

        elapsed += check_interval;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Sequencia de disparos do rele %u concluida com sucesso.",
        relay_id
    );

    return BT::NodeStatus::SUCCESS;
}

void FireNode::halt()
{
    halt_requested_ = true;
}
