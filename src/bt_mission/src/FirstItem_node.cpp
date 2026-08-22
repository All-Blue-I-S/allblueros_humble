#include "FirstItem_node.h"

BT::NodeStatus FirstItem::tick()
{
    int count;

    if (!getInput<int>("counter", count))
    {
        throw BT::RuntimeError("missing port [counter]");
    }

    return (count == 0)
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
}
