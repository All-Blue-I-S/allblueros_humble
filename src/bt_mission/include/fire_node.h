#ifndef FIRE_NODE_H
#define FIRE_NODE_H

#include <atomic>

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/srv/command_long.hpp>

class FireNode : public BT::AsyncActionNode
{
private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr mavros_client_;

    std::atomic_bool halt_requested_;

public:
    FireNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    BT::NodeStatus tick() override;

    void halt() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<unsigned int>(
                "relay_id",
                "ID do relé na Pixhawk (ex: 0 para AUX1)"
            ),

            BT::InputPort<int>(
                "cycles",
                "Número de vezes que o solenoide vai bater"
            ),

            BT::InputPort<float>(
                "cycle_time",
                "Duração de cada ciclo em segundos"
            )
        };
    }
};

#endif // FIRE_NODE_H
