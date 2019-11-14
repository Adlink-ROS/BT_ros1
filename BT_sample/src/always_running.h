#pragma once

#include <behaviortree_cpp_v3/action_node.h>

class AlwaysRunning : public BT::AsyncActionNode
{
    public:
        AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
        }

        static BT::PortsList providedPorts()
        {
            return{};
        }

        virtual BT::NodeStatus tick() override
        {
            printf("Running......\n");
            return BT::NodeStatus::RUNNING;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
};
