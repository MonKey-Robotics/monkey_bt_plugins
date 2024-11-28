#include "monkey_bt_plugins/plugins/btros2_services/arm_inverse_kinematics.hpp"
#include "behaviortree_ros2/plugins.hpp"

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool ArmInverseKinematics::setRequest(Request::SharedPtr& request)
    {
      // must return true if we are ready to send the request
      getInput<double>("target_x", request->target.x);
      getInput<double>("target_y", request->target.y);
      getInput<double>("target_z", request->target.z);

      // Logging
      std::stringstream ss;
      ss << this->name() << " -> x: " << request->target.x 
        << ", y: " << request->target.y
        << ", z: " << request->target.z;
      RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    BT::NodeStatus ArmInverseKinematics::onResponseReceived(const Response::SharedPtr& response)
    {
      // Log
      std::stringstream ss;
      ss << this->name() << " -> b0: " << response->b0_target*180/M_PI
        << ", b1: " << response->b1_target
        << ", b2: " << response->b2_target;
      RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

      if (!response->success) {
        return BT::NodeStatus::FAILURE;
      }
      
      setOutput("b0_target", response->b0_target*180/M_PI);
      setOutput("b1_target", response->b1_target);
      setOutput("b2_target", response->b2_target);
      return BT::NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    BT::NodeStatus ArmInverseKinematics::onFailure(BT::ServiceNodeErrorCode error)
    {
      std::stringstream ss;
      ss << this->name() << " -> Error: " << error;
      RCLCPP_INFO(ros2_node->get_logger(), ss.str().c_str());

      return BT::NodeStatus::FAILURE;
    }

CreateRosNodePlugin(ArmInverseKinematics, "ArmInverseKinematics");