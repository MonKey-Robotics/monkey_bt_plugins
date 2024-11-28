#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_action_node.hpp"
#include "arm_msgs/action/home_request.hpp"

#pragma once
using namespace BT;

class HomeRequestAction: public RosActionNode<arm_msgs::action::HomeRequest>
{
private:
  std::shared_ptr<rclcpp::Node> ros2_node;
public:
HomeRequestAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<arm_msgs::action::HomeRequest>(name, conf, params)
  {
    ros2_node = node_.lock();
  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("action_name")
    });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(Goal& goal) override;

  // Based on the reply you may decide to return SUCCESS or FAILURE.
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  
  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback>);

};