#include "monkey_bt_plugins/plugins/btros2_services/get_position_service.hpp"
#include "behaviortree_ros2/plugins.hpp"

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool GetPositionService::setRequest(Request::SharedPtr& request)
    {
      // must return true if we are ready to send the request
      (void)request;
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    BT::NodeStatus GetPositionService::onResponseReceived(const Response::SharedPtr& response)
    {
      // Log
      std::stringstream ss;
      ss << " -> Position: " << response->position;
      RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

      setOutput("position", response->position);
      return BT::NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    BT::NodeStatus GetPositionService::onFailure(BT::ServiceNodeErrorCode error)
    {
      std::stringstream ss;
      ss << ros2_node->get_name() << " -> Error: " << error;
      RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

      return BT::NodeStatus::FAILURE;
    }

// Plugin registration.
// The class SleepAction will self register with name  "SleepAction".
CreateRosNodePlugin(GetPositionService, "GetPositionService");