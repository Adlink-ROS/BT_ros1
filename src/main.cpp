#include "movebase_client.h"
#include "chk_low_battery.h"
#include "always_running.h"
#include "interrupt_event.h"
#include "send_cmd_vel.h"
#ifdef SUPPORT_OPENVINO
  #include "openvino_event.h"
#endif
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_bt");

  ros::NodeHandle nh("~");
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "/home/ros/catkin_kk_ws/src/BT_ros1/BT_sample/cfg/bt_test.xml");
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<SendCommandVel>("SendCommandVel");
  factory.registerSimpleCondition("CheckBattery", CheckBattery, {BT::InputPort<int>("wait_tick")});
  factory.registerNodeType<AlwaysRunning>("AlwaysRunning");
  factory.registerNodeType<InterruptEvent>("InterruptEvent");
#ifdef SUPPORT_OPENVINO
  factory.registerNodeType<OpenVINOEvent>("OpenVINOEvent");
#endif

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    status = tree.rootNode()->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
