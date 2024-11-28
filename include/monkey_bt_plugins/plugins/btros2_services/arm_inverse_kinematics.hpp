#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_service_node.hpp"
#include "arm_msgs/srv/inverse_kinematics.hpp"

#pragma once
using namespace BT;

class ArmInverseKinematics: public RosServiceNode<arm_msgs::srv::InverseKinematics>
{
  private:
    std::shared_ptr<rclcpp::Node> ros2_node;
  public:
    ArmInverseKinematics(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<arm_msgs::srv::InverseKinematics>(name, conf, params)
    {
      ros2_node = node_.lock();
    }

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<std::string>("service_name"),
        InputPort<double>("target_x"),
        InputPort<double>("target_y"),
        InputPort<double>("target_z"),
        OutputPort<double>("b0_target"),
        OutputPort<double>("b1_target"),
        OutputPort<double>("b2_target")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override;

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};