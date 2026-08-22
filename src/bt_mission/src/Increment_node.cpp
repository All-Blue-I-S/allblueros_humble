#include "Increment_node.h"

BT::NodeStatus Increment::tick()
{
    int count = 0;

    getInput("counter", count);

    count++;

    setOutput("counter", count);

    return BT::NodeStatus::SUCCESS;
}
