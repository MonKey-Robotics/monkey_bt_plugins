#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_service_node.hpp"
#include "hand_interfaces/srv/frond_position.hpp"

#pragma once
using namespace BT;


class FrondPositionService : public BT::RosServiceNode<hand_interfaces::srv::FrondPosition>
{
private:
  std::string service_suffix_;
  std::shared_ptr<rclcpp::Node> ros2_node;
public:
  explicit FrondPositionService(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
    : RosServiceNode<hand_interfaces::srv::FrondPosition>(name, conf, params)
  {
    ros2_node = node_.lock(); //VERY IMPORTANT >> To make it into shared/strong ptr
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_x"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_y"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_z")
      });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};