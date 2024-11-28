#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_action_node.hpp"
#include "cutting_planner_interfaces/action/hold_up_home.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#pragma once
using namespace BT;

class HoldUpHome: public RosActionNode<cutting_planner_interfaces::action::HoldUpHome>
{
  private:
    rclcpp::Node::SharedPtr ros2_node;
  public:
    HoldUpHome(const std::string& name, 
                    const NodeConfig& conf, 
                    const RosNodeParams& params) 
    : RosActionNode<cutting_planner_interfaces::action::HoldUpHome>(name, conf, params)
    {
      ros2_node = node_.lock();
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({
        BT::InputPort<std::string>("action_name"),
        BT::InputPort<uint16_t>("task"),
        BT::OutputPort<bool>("success")
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
};