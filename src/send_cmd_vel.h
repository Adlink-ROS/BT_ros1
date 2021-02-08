#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <actionlib/client/simple_action_client.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class SendCommandVel : public BT::AsyncActionNode
{
  public:
    SendCommandVel(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        // Init cmd_vel publisher
        pub_ = node_.advertise<geometry_msgs::Twist>("/pimir/cmd_vel", 1);
    }

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("movement") };
    }

    virtual BT::NodeStatus tick() override;

  private:
    ros::NodeHandle node_;
    ros::Publisher pub_;
};

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Init variables
float speed(0.5); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

BT::NodeStatus SendCommandVel::tick()
{
  // Create Twist message
  geometry_msgs::Twist twist;
  std::string movement;
  getInput<std::string>("movement", movement);
  auto parts = BT::splitString(movement, ';');
  std::string key = BT::convertFromString<std::string>(parts[0]);
  char move_key= key[0];
  double time_input = BT::convertFromString<double>(parts[1]);
  uint32_t time = time_input*1000000;
  //fprintf(stderr, "time['%d']\n", time);
  
   // If the key corresponds to a key in moveBindings
  if (moveBindings.count(move_key) == 1)
  {
    // Grab the direction data
    x = moveBindings[move_key][0];
    y = moveBindings[move_key][1];
    z = moveBindings[move_key][2];
    th = moveBindings[move_key][3];
  }
  // Otherwise if it corresponds to a key in speedBindings
  else if (speedBindings.count(move_key) == 1)
  {
    // Grab the speed data
    speed = speed * speedBindings[move_key][0];
    turn = turn * speedBindings[move_key][1];  
  }
  // Otherwise, set the robot to stop
  else
  {
    x = 0;
    y = 0;
    z = 0;
    th = 0;    
  }

  // Update the Twist message
  twist.linear.x = x * speed;
  twist.linear.y = y * speed;
  twist.linear.z = z * speed;    
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = th * turn;    

  // Publish it and resolve any remaining callbacks
  pub_.publish(twist);
  ros::spinOnce();
  usleep(time);
  return BT::NodeStatus::SUCCESS;
}

