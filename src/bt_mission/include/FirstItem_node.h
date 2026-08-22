#ifndef FIRST_ITEM_NODE_H
#define FIRST_ITEM_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

class FirstItem : public BT::ConditionNode
{
public:

    FirstItem(
        const std::string& name,
        const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("counter")
        };
    }
};

#endif  // FIRST_ITEM_NODE_H
