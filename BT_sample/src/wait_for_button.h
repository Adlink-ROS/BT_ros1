#pragma once

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <behaviortree_cpp_v3/action_node.h>

class WaitForButton : public BT::AsyncActionNode
{
    public:
        WaitForButton(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
            _go_switch = false;
            ros::NodeHandle n;
            sub = n.subscribe("go_switch", 1000, &WaitForButton::cb, this);
        }

        static BT::PortsList providedPorts()
        {
            return{};
        }

        virtual BT::NodeStatus tick() override
        {
            // ROS_INFO("I was done here");
            
            /*
            while(ros::ok)                                                                                                                                               
            {                                                                                                                                                            
                ros::spinOnce;                                                                                                                                       
                cout << myNode.y[0] << endl << myNode.y[1] << endl;                                                                                                  
                r.sleep();                                                                                                                                           
            } 
            */ 
            ros::Rate r(10);
            while (!_go_switch && ros::ok())
            {
                // ROS_INFO(_go_switch ? "True" : "False");
                if (_go_switch) {
                    break;
                    // ROS_INFO("Success here");
                } else {
                    ROS_INFO("Waiting for the call");
                    // return BT::NodeStatus::RUNNING;
                }
                ros::spinOnce();
                r.sleep();
            }
            
            return BT::NodeStatus::SUCCESS;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

        void cb(const std_msgs::Bool::ConstPtr& msg)
        {
            _go_switch = msg->data;
            ROS_INFO(msg->data ? "True" : "False");
        }

    private:
        bool _aborted;
        bool _go_switch;
        ros::Subscriber sub;
};
