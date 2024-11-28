#include "monkey_bt_plugins/plugins/btros2_actions/set_target_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

// This is called when the TreeNode is ticked and it should
// send the request to the action server
bool SetTargetAction::setGoal(RosActionNode::Goal& goal) 
{
// get "target" from the Input port
getInput("target", goal.target);
// return true, if we were able to set the goal correctly.
return true;
}

// Callback executed when the reply is received.
// Based on the reply you may decide to return SUCCESS or FAILURE.
BT::NodeStatus SetTargetAction::onResultReceived(const WrappedResult& wr)
{ 
std::stringstream ss;
bool target_condition = wr.result->success;

if (target_condition == true)
{
ss << this->name() << " -> SetTarget SUCCESS";
RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
return BT::NodeStatus::SUCCESS;
} else {
ss << this->name() << " -> SetTarget FAILED";
RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
return BT::NodeStatus::FAILURE;
}
}

// Callback invoked when there was an error at the level
// of the communication between client and server.
// This will set the status of the TreeNode to either SUCCESS or FAILURE,
// based on the return value.
// If not overridden, it will return FAILURE by default.
BT::NodeStatus SetTargetAction::onFailure(BT::ActionNodeErrorCode error)
{
std::stringstream ss;
ss << this->name() << " -> Error: " << error;
RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

return BT::NodeStatus::FAILURE;
}

// we also support a callback for the feedback, as in
// the original tutorial.
// Usually, this callback should return RUNNING, but you
// might decide, based on the value of the feedback, to abort
// the action, and consider the TreeNode completed.
// In that case, return SUCCESS or FAILURE.
// The Cancel request will be send automatically to the server.
BT::NodeStatus SetTargetAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
setOutput("position", feedback->current_position);
// Logging
std::stringstream ss;
ss << this->name() << " -> Current position: " << feedback->current_position;
RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

return BT::NodeStatus::RUNNING;
}

CreateRosNodePlugin(SetTargetAction, "SetTargetAction");