#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "object_msgs/ObjectsInBoxes.h"

static std::vector<std::string> detected_objects;

void OpenVINOCallback(const object_msgs::ObjectsInBoxes::ConstPtr msg)
{
    int cnt = 0;
    for(auto & obj : msg->objects_vector)
    {
        std::cout << "["<< ++cnt <<"] Detect object: '" << obj.object.object_name << "'." << std::endl;
        detected_objects.push_back(obj.object.object_name);
    }
}

class OpenVINOEvent : public BT::SyncActionNode
{
    public:
        OpenVINOEvent(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            sub_ = node_.subscribe("/ros_openvino_toolkit/detected_objects", 1000, OpenVINOCallback);
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("object") };
        }

        virtual BT::NodeStatus tick() override
        {
            std::string expect_object;

            if (!getInput<std::string>("object", expect_object)) {
                throw BT::RuntimeError("missing required input [object]");
            }

            detected_objects.clear();
            ros::spinOnce();

            std::vector<std::string>::iterator it;
            it = find(detected_objects.begin(), detected_objects.end(), expect_object);
            if (it == detected_objects.end())
                return BT::NodeStatus::FAILURE;
            else
                return BT::NodeStatus::SUCCESS;
        }

    private:
        ros::NodeHandle node_;
        ros::Subscriber sub_;
};
