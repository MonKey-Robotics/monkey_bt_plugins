#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_service_node.hpp"
#include "monkey_std_msgs/srv/get_position.hpp"

#pragma once
using namespace BT;


class GetPositionService : public BT::RosServiceNode<monkey_std_msgs::srv::GetPosition>
{
private:
  std::string service_suffix_;
  std::shared_ptr<rclcpp::Node> ros2_node;
public:
  explicit GetPositionService(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
    : RosServiceNode<monkey_std_msgs::srv::GetPosition>(name, conf, params)
  {
    ros2_node = node_.lock(); //VERY IMPORTANT >> To make it into shared/strong ptr
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::OutputPort<double>("position")
      });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};