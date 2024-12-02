#include "monkey_bt_plugins/plugins/btros2_actions/cutter_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool CutterControl::setGoal(RosActionNode::Goal& goal)
{
  // get "task" from the Input port
  getInput("command", goal.command);
  // return true, if we were able to set the goal correctly.
  return true;
}

// Callback executed when the reply is received.
// Based on the reply you may decide to return SUCCESS or FAILURE.
BT::NodeStatus CutterControl::onResultReceived(const WrappedResult& wr)
{
  std::stringstream ss;
  
  if (wr.result->success != true)
  {
    ss << this->name() << " -> failed";
    RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ss << this->name() << " -> succeeded";
    RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
    return BT::NodeStatus::SUCCESS;
  }
}

// Callback invoked when there was an error at the level
// of the communication between client and server.
// This will set the status of the TreeNode to either SUCCESS or FAILURE,
// based on the return value.
// If not overridden, it will return FAILURE by default.
BT::NodeStatus CutterControl::onFailure(BT::ActionNodeErrorCode error)
{
  std::stringstream ss;
  ss << this->name() << " -> Error: " << error;
  RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

  return BT::NodeStatus::FAILURE;
}

CreateRosNodePlugin(CutterControl, "CutterControl");