#include "monkey_bt_plugins/plugins/btros2_actions/home_request_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool HomeRequestAction::setGoal(RosActionNode::Goal& goal)
{
  // must return true if we are ready to send the request
  RCLCPP_INFO(ros2_node->get_logger(), "Homing Request Sent");
  (void)goal;
  return true;
}

// Callback invoked when the answer is received.
// It must return SUCCESS or FAILURE
BT::NodeStatus HomeRequestAction::onResultReceived(const WrappedResult& wr)
{
  bool home_status = wr.result->homing_status;
  std::stringstream ss;
  if (home_status == true)
  {
    // Log
    ss << this->name() << " -> Homing Status: SUCCESS";
    RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
  } else {
    // Log
    ss << this->name() << " -> Homing Status: FAILURE";
    RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
    return NodeStatus::FAILURE;
  }
  
  //return NodeStatus::FAILURE; <-- If error here just add this
}

BT::NodeStatus HomeRequestAction::onFailure(BT::ActionNodeErrorCode error)
{
  std::stringstream ss;
  ss << this->name() << " -> Error: " << error;
  RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());
  return NodeStatus::FAILURE;
}

BT::NodeStatus HomeRequestAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    // Logging
    std::stringstream ss;
    ss << this->name() << " -> Current position: " << feedback->current_position;
    RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

    return BT::NodeStatus::RUNNING;
  }

CreateRosNodePlugin(HomeRequestAction, "HomeRequestAction");