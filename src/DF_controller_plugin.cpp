#include "DF_controller_plugin.hpp"

namespace controller_plugin_differential_flatness
{

  void Plugin::ownInitialize()
  {
    flags_.parameters_read = false;
    flags_.state_received = false;
    flags_.ref_received = false;

    controller_handler_ = std::make_shared<DFController>();

    static auto parameters_callback_handle_ = node_ptr_->add_on_set_parameters_callback(
        std::bind(&Plugin::parametersCallback, this, std::placeholders::_1));

    declareParameters();

    resetState();
    resetReferences();
    resetCommands();
  };

  void Plugin::updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                           const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    uav_state_.pos = Vector3d(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z);

    uav_state_.vel = Vector3d(
        twist_msg.twist.linear.x,
        twist_msg.twist.linear.y,
        twist_msg.twist.linear.z);

    tf2::Quaternion q(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w);

    uav_state_.rot = q;

    flags_.state_received = true;
    return;
  };

  void Plugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg)
  {

    if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION)
    {
      control_ref_.pos = Vector3d(
          pose_msg.pose.position.x,
          pose_msg.pose.position.y,
          pose_msg.pose.position.z);

      flags_.ref_received = true;
    }

    if ((control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
         control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) &&
        control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE)
    {
      tf2::Quaternion q(
          pose_msg.pose.orientation.x,
          pose_msg.pose.orientation.y,
          pose_msg.pose.orientation.z,
          pose_msg.pose.orientation.w);

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      control_ref_.yaw[0] = yaw;
    }
    return;
  };

  void Plugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION)
    {
      speed_limits_ = Vector3d(
          twist_msg.twist.linear.x,
          twist_msg.twist.linear.y,
          twist_msg.twist.linear.z);
      return;
    }

    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED)
    {
      return;
    }

    control_ref_.vel = Vector3d(
        twist_msg.twist.linear.x,
        twist_msg.twist.linear.y,
        twist_msg.twist.linear.z);

    if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED)
    {
      control_ref_.yaw[1] = twist_msg.twist.angular.z;
    }

    flags_.ref_received = true;
    return;
  };

  void Plugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY)
    {
      return;
    }

    control_ref_.pos = Vector3d(
        traj_msg.positions[0],
        traj_msg.positions[1],
        traj_msg.positions[2]);

    control_ref_.vel = Vector3d(
        traj_msg.velocities[0],
        traj_msg.velocities[1],
        traj_msg.velocities[2]);

    control_ref_.acc = Vector3d(
        traj_msg.accelerations[0],
        traj_msg.accelerations[1],
        traj_msg.accelerations[2]);

    control_ref_.yaw = Vector3d(
        traj_msg.positions[3],
        traj_msg.velocities[3],
        traj_msg.accelerations[3]);

    flags_.ref_received = true;
    return;
  };

  void Plugin::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                             geometry_msgs::msg::TwistStamped &twist,
                             as2_msgs::msg::Thrust &thrust)
  {
    if (!flags_.state_received)
    {
      RCLCPP_WARN_ONCE(node_ptr_->get_logger(), "State not received yet");
      return;
    }

    if (!flags_.parameters_read)
    {
      RCLCPP_WARN_ONCE(node_ptr_->get_logger(), "Parameters not read yet");
      return;
    }

    if (!flags_.ref_received)
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "State changed, but ref not recived yet");
    }

    computeActions(pose, twist, thrust);
    static rclcpp::Time last_time_ = node_ptr_->now();
    return;
  };

  bool Plugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                       const as2_msgs::msg::ControlMode &out_mode)
  {
    control_mode_in_ = in_mode;
    static auto last_mode = control_mode_in_;
    control_mode_out_ = out_mode;

    flags_.ref_received = false;
    flags_.state_received = false;
    in_hover_ = false;

    if (control_mode_in_.control_mode != last_mode.control_mode)
    {
      controller_handler_->resetError();
    }

    resetReferences();

    last_time_ = node_ptr_->now();
    last_mode = control_mode_in_;

    return true;
  };

  rcl_interfaces::msg::SetParametersResult Plugin::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (auto &param : parameters)
    {
      if (controller_handler_->isParameter(param.get_name()))
      {
        controller_handler_->setParameter(param.get_name(), param.get_value<double>());

        // Remove the parameter from the list of parameters to be read
        parameters_to_read_.erase(
            std::remove(parameters_to_read_.begin(), parameters_to_read_.end(), param.get_name()),
            parameters_to_read_.end());
        if (parameters_to_read_.empty())
        {
          RCLCPP_INFO(node_ptr_->get_logger(), "All parameters read");
          flags_.parameters_read = true;
        }
      }
      else
      {
        RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not defined in controller params",
                    param.get_name().c_str());
        result.successful = false;
        result.reason = "parameter not found";
      }
    }
    return result;
  };

  void Plugin::declareParameters()
  {
    std::vector<std::pair<std::string, double>> params = controller_handler_->getParametersList();
    for (auto &param : params)
    {
      node_ptr_->declare_parameter(param.first);
    }
    return;
  };

  void Plugin::computeActions(geometry_msgs::msg::PoseStamped &pose,
                              geometry_msgs::msg::TwistStamped &twist,
                              as2_msgs::msg::Thrust &thrust)
  {
    resetCommands();

    rclcpp::Time current_time = node_ptr_->now();
    double dt = (current_time - last_time_).nanoseconds() / 1.0e9;
    last_time_ = current_time;

    switch (control_mode_in_.control_mode)
    {
    case as2_msgs::msg::ControlMode::HOVER:
      f_des_ = controller_handler_->computePositionControl(uav_state_, hover_ref_, dt, speed_limits_);
      break;
    case as2_msgs::msg::ControlMode::SPEED:
    {
      if (control_ref_.vel.norm() < 1e-4)
      {
        if (!in_hover_)
        {
          in_hover_ = true;
          resetReferences();
          controller_handler_->resetError();
        }
        f_des_ = controller_handler_->computePositionControl(uav_state_, hover_ref_, dt, speed_limits_);
      }
      else
      {
        in_hover_ = false;
        f_des_ = controller_handler_->computeVelocityControl(uav_state_, control_ref_, dt);
      }
      break;
    }
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      f_des_ = controller_handler_->computeTrajectoryControl(uav_state_, control_ref_, dt);
      break;
    default:
      RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Unknown control mode");
      return;
      break;
    }

    switch (control_mode_in_.yaw_mode)
    {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      controller_handler_->computeYawAngleControl(
          // Input
          uav_state_, control_ref_.yaw[0], f_des_,
          // Output
          acro_, thrust_);
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
    {
      controller_handler_->computeYawSpeedControl(
          // Input
          uav_state_, control_ref_.yaw[1], f_des_, dt,
          // Output
          acro_, thrust_);
      break;
    }
    default:
      RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Unknown yaw mode");
      return;
      break;
    }

    switch (control_mode_in_.reference_frame)
    {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      getOutput(pose, twist, thrust);
      break;

    default:
      RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Unknown reference frame");
      return;
      break;
    }

    return;
  };

  void Plugin::getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                         geometry_msgs::msg::TwistStamped &twist_msg,
                         as2_msgs::msg::Thrust &thrust_msg)
  {
    twist_msg.header.stamp = node_ptr_->now();

    twist_msg.twist.angular.x = acro_(0);
    twist_msg.twist.angular.y = acro_(1);
    twist_msg.twist.angular.z = acro_(2);

    thrust_msg.header.stamp = node_ptr_->now();

    thrust_msg.thrust = thrust_;

    return;
  };

  void Plugin::resetState()
  {
    uav_state_.pos = Vector3d::Zero();
    uav_state_.vel = Vector3d::Zero();
    uav_state_.rot = tf2::Quaternion::getIdentity();
    return;
  };

  void Plugin::resetReferences()
  {
    control_ref_.pos = uav_state_.pos;
    control_ref_.vel = Vector3d::Zero();
    control_ref_.acc = Vector3d::Zero();

    speed_limits_ = Vector3d::Zero();

    tf2::Matrix3x3 m_input(uav_state_.rot);
    double roll, pitch, yaw;
    m_input.getRPY(roll, pitch, yaw);

    control_ref_.yaw = Vector3d(yaw, 0.0f, 0.0f);

    hover_ref_ = control_ref_;

    return;
  };

  void Plugin::resetCommands()
  {
    f_des_.setZero();
    acro_.setZero();
    thrust_ = 0.0f;
    return;
  };

} // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_differential_flatness::Plugin,
                       controller_plugin_base::ControllerBase)
