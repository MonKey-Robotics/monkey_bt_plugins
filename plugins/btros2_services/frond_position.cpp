#include "monkey_bt_plugins/plugins/btros2_services/frond_position.hpp"
#include "behaviortree_ros2/plugins.hpp"

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
bool FrondPositionService::setRequest(Request::SharedPtr& request)
    {
      // must return true if we are ready to send the request
      (void)request;
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
BT::NodeStatus FrondPositionService::onResponseReceived(const Response::SharedPtr& response)
    {
      // Log
      std::stringstream ss;
      // ss << " -> Position : " << response->pose;
      RCLCPP_INFO(ros2_node->get_logger(), "Position x: %.2f, y: %.2f, z: %.2f", 
        response->pose.pose.position.x, response->pose.pose.position.y, response->pose.pose.position.z);

      setOutput("pose", response->pose);
      setOutput("pose_x", response->pose.pose.position.x);
      setOutput("pose_y", response->pose.pose.position.y);
      setOutput("pose_z", response->pose.pose.position.z);

      if (response->success) {
        return BT::NodeStatus::SUCCESS;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
BT::NodeStatus FrondPositionService::onFailure(BT::ServiceNodeErrorCode error)
    {
      std::stringstream ss;
      ss << ros2_node->get_name() << " -> Error: " << error;
      RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

      return BT::NodeStatus::FAILURE;
    }

// Plugin registration.
// The class SleepAction will self register with name  "SleepAction".
CreateRosNodePlugin(FrondPositionService, "FrondPositionService");