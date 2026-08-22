#ifndef INCREMENT_NODE_H
#define INCREMENT_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

class Increment : public BT::SyncActionNode
{
public:
    Increment(
        const std::string& name,
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::BidirectionalPort<int>("counter")
        };
    }
};

#endif  // INCREMENT_NODE_H
