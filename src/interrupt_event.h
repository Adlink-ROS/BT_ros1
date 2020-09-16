#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "std_msgs/String.h"

static std::string interrupt_event;

void interruptCallback(const std_msgs::String::ConstPtr& msg)
{
    printf("interrupt callback detect event: %s\n", msg->data.c_str());
    interrupt_event = msg->data;
}

class InterruptEvent : public BT::SyncActionNode
{
    public:
        InterruptEvent(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            sub_ = node_.subscribe("interrupt_event", 1000, interruptCallback);
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("event") };
        }

        virtual BT::NodeStatus tick() override
        {
            std::string expect_event;

            //printf("Checking interrupt....\n");
            if (!getInput<std::string>("event", expect_event)) {
                throw BT::RuntimeError("missing required input [event]");
            }

            interrupt_event = "";
            ros::spinOnce();

            if (interrupt_event == expect_event)
                return BT::NodeStatus::FAILURE;
            else
                return BT::NodeStatus::SUCCESS;
        }

    private:
        ros::NodeHandle node_;
        ros::Subscriber sub_;
};
