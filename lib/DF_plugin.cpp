#include "DF_plugin.hpp"

#pragma region DF_controller

namespace differential_flatness_controller
{
  DFController::DFController()
  {
    resetError();
  };


  bool DFController::setParameter(const std::string &param, const double &value)
  {
    if (parameters_.count(param) == 1)
    {
      parameters_[param] = value;
      updateGains_();
      return true;
    }
    return false;
  };

  bool DFController::getParameter(const std::string &param, double &value)
  {
    if (parameters_.count(param) == 1)
    {
      value = parameters_[param];
      return true;
    }
    return false;
  };

  bool DFController::isParameter(const std::string &param)
  {
    return parameters_.count(param) == 1;
  };

  bool DFController::setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list)
  {
    for (auto &param : parameter_list)
    {
      if (parameters_.count(param.first) == 1)
      {
        parameters_[param.first] = param.second;
      }
      else
      {
        return false;
      }
    }
    updateGains_();
    return true;
  };

  std::vector<std::pair<std::string,double>> DFController::getParametersList()
  {
    std::vector<std::pair<std::string,double>> list;
    for (auto& param : parameters_)
    {
      list.push_back({param.first, param.second});
    }
    return list;
  }; 

  Eigen::Vector3d DFController::getPositionError()
  {
    return position_accum_error_;
  };

  Eigen::Vector3d DFController::getVelocityError()
  {
    return position_accum_error_;
  };

  void DFController::resetError()
  {
    position_accum_error_.setZero();
    velocity_accum_error_.setZero();
    return;
  };
  
  // Compute the velocity control by velocity references
  Vector3d DFController::computeVelocityControl(
    const UAV_state &state_,
    const Control_ref &ref,
    const double &dt
  ){
    // Compute the proportional contribution (velocity)
    Vector3d velocity_error = ref.vel - state_.vel;
    Vector3d p_velocity_error_contribution = velocity_Kp_lin_mat_ * velocity_error;

    // Store the error for the next iteration
    static Vector3d last_velocity_error_ = velocity_error;
    static Vector3d filtered_d_velocity_error_ = velocity_error;

    // Compute the derivative contribution of the error filtered with a first order filter (velocity derivate) 
    Vector3d velocity_error_incr = (velocity_error - last_velocity_error_);
    filtered_d_velocity_error_ = alpha_ * velocity_error_incr + (1.0 - alpha_) * filtered_d_velocity_error_;
    Vector3d d_velocity_error_contribution = velocity_Kd_lin_mat_ * filtered_d_velocity_error_/ dt;

    // Update the last velocity error
    last_velocity_error_ = velocity_error;

    // Update de acumulated error
    velocity_accum_error_ += velocity_error * dt;

    // Compute anti-windup. Limit integral contribution
    for (short j = 0; j < 3; j++) {
      float antiwindup_value = antiwindup_cte_ / velocity_Ki_lin_mat_.diagonal()[j];
      velocity_accum_error_[j] = (velocity_accum_error_[j] >  antiwindup_value) ? antiwindup_value : velocity_accum_error_[j];
      velocity_accum_error_[j] = (velocity_accum_error_[j] < -antiwindup_value) ? -antiwindup_value : velocity_accum_error_[j];
    }

    // Compute de integral contribution (velocity integrate)
    Vector3d i_velocity_error_contribution = velocity_Ki_lin_mat_ * velocity_accum_error_;

    // Compute desired acceleration
    Vector3d desired_acceleration = p_velocity_error_contribution + d_velocity_error_contribution + i_velocity_error_contribution;

    // Compute gravity compensation
    Vector3d force_gravity = mass_ * gravitational_accel_;

    // Return desired force with the gravity compensation
    Vector3d desired_force = mass_ * desired_acceleration + force_gravity;
    return desired_force;
  };


  Vector3d DFController::computeTrajectoryControl(
    const UAV_state &state,
    const Control_ref &ref,
    const double &dt
  ){
    // Compute the proportional contribution (position) 
    Vector3d position_error = ref.pos - state.pos;
    Vector3d p_error_contribution = traj_Kp_lin_mat_ * position_error;

    // Compute de derivative error contribution (velocity)
    Vector3d velocity_error = ref.vel - state.vel;
    Vector3d d_error_contribution = traj_Kd_lin_mat_ * velocity_error;

    // Update de acumulated error
    position_accum_error_ += position_error;

    // Compute anti-windup. Limit integral contribution
    for (short j = 0; j < 3; j++) {
      float antiwindup_value = antiwindup_cte_ / traj_Ki_lin_mat_.diagonal()[j];
      position_accum_error_[j] = (position_accum_error_[j] >  antiwindup_value) ? antiwindup_value : position_accum_error_[j];
      position_accum_error_[j] = (position_accum_error_[j] < -antiwindup_value) ? -antiwindup_value : position_accum_error_[j];
    }

    // Compute de integral contribution (position integrate)
    Vector3d i_error_contribution = traj_Ki_lin_mat_ * position_accum_error_;

    // Compute the error force contribution
    Vector3d force_error = p_error_contribution + d_error_contribution + i_error_contribution;

    // Compute acceleration reference contribution
    Vector3d force_acceleration = mass_ * ref.acc;

    // Compute gravity compensation
    Vector3d force_gravity = mass_ * gravitational_accel_;

    // Return desired force with the gravity compensation
    Vector3d desired_force = - force_error + force_acceleration + force_gravity;
    return desired_force;
  }

  void DFController::computeYawAngleControl(
    // Input
    const UAV_state &state,
    const float &yaw_angle_ref,
    const Vector3d &force_des,
    // Output
    Vector3d &acro, 
    float &thrust
  ) {

    Vector3d xc_des(cos(yaw_angle_ref), sin(yaw_angle_ref), 0);

    Vector3d zb_des = force_des.normalized();
    Vector3d yb_des = zb_des.cross(xc_des).normalized();
    Vector3d xb_des = yb_des.cross(zb_des).normalized();

    // Compute the rotation matrix desidered
    Eigen::Matrix3d R_des;
    R_des.col(0) = xb_des;
    R_des.col(1) = yb_des;
    R_des.col(2) = zb_des;

    // Compute the rotation matrix error
    Eigen::Matrix3d Mat_e_rot = (R_des.transpose() * state.rot - state.rot.transpose() * R_des);

    Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
    Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

    thrust = (float)force_des.dot(state.rot.col(2).normalized());

    Vector3d outputs = -Kp_ang_mat_ * E_rot;

    acro[0] = outputs(0);  // ROLL
    acro[1] = outputs(1);  // PITCH
    acro[2] = outputs(2);  // YAW

    return;
  };

  void DFController::computeYawSpeedControl(
    // Input
    const UAV_state &state,
    const float &yaw_speed_ref,
    const Vector3d &force_des,
    const double &dt,
    // Output
    Vector3d &acro, 
    float &thrust
  ) {

    float yaw_state = state.rot.eulerAngles(0, 1, 2)[2];
    float yaw_angle_ref = yaw_state + yaw_speed_ref * dt;
    
    computeYawAngleControl(
      state,
      yaw_angle_ref,
      force_des,
      acro, 
      thrust
    );

    return;
  };

  void DFController::updateGains_()
  {
    mass_ = parameters_["mass"];
    antiwindup_cte_ = parameters_["antiwindup_cte"];
    alpha_ = parameters_["alpha"];

    traj_Kp_lin_mat_ = Vector3d(
      parameters_["trajectory_following.position_Kp.x"], 
      parameters_["trajectory_following.position_Kp.y"], 
      parameters_["trajectory_following.position_Kp.z"]
    ).asDiagonal();

    traj_Ki_lin_mat_ = Vector3d(
      parameters_["trajectory_following.position_Ki.x"], 
      parameters_["trajectory_following.position_Ki.y"], 
      parameters_["trajectory_following.position_Ki.z"]
    ).asDiagonal();
    
    traj_Kd_lin_mat_ = Vector3d(
      parameters_["trajectory_following.position_Kd.x"], 
      parameters_["trajectory_following.position_Kd.y"], 
      parameters_["trajectory_following.position_Kd.z"]
    ).asDiagonal();

    velocity_Kp_lin_mat_ = Vector3d(
      parameters_["speed_following.speed_Kp.x"], 
      parameters_["speed_following.speed_Kp.y"], 
      parameters_["speed_following.speed_Kp.z"]
    ).asDiagonal();

    velocity_Ki_lin_mat_ = Vector3d(
      parameters_["speed_following.speed_Ki.x"], 
      parameters_["speed_following.speed_Ki.y"], 
      parameters_["speed_following.speed_Ki.z"]
    ).asDiagonal();

    velocity_Kd_lin_mat_ = Vector3d(
      parameters_["speed_following.speed_Kd.x"], 
      parameters_["speed_following.speed_Kd.y"], 
      parameters_["speed_following.speed_Kd.z"]
    ).asDiagonal();

    Kp_ang_mat_ = Vector3d(
      parameters_["angular_speed_controller.angular_gain.x"], 
      parameters_["angular_speed_controller.angular_gain.y"], 
      parameters_["angular_speed_controller.angular_gain.z"]
    ).asDiagonal();

    return;
  };
}

#pragma endregion

#pragma region DF_PLUGIN

namespace controller_plugin_differential_flatness
{

  void DFPlugin::ownInitialize()
  {
    flags_.parameters_read = false;
    flags_.state_received = false;
    flags_.ref_received = false;

    controller_handler_ = std::make_shared<DFController>();


    flags_.parameters_read  = true;
    // static auto parameters_callback_handle_ = node_ptr_->add_on_set_parameters_callback(
    //   std::bind(&DFPlugin::parametersCallback, this, std::placeholders::_1));

    // declareParameters();

    resetState();
    resetReferences();
    resetCommands();
  };

  void DFPlugin::updateState(const nav_msgs::msg::Odometry &odom)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "DFPlugin::updateState");
    uav_state_.pos = Vector3d(
      odom.pose.pose.position.x, 
      odom.pose.pose.position.y, 
      odom.pose.pose.position.z
    );

    uav_state_.vel = Vector3d(
      odom.twist.twist.linear.x, 
      odom.twist.twist.linear.y, 
      odom.twist.twist.linear.z
    );

    Eigen::Quaterniond q(
      odom.pose.pose.orientation.w, 
      odom.pose.pose.orientation.x, 
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z
    );

    uav_state_.rot = q.toRotationMatrix();

    flags_.state_received = true;
    return;
  };

  void DFPlugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED)
    {
      return;
    }

    RCLCPP_INFO(node_ptr_->get_logger(), "DFPlugin::updateReference");

    control_ref_.vel = Vector3d(
      twist_msg.twist.linear.x, 
      twist_msg.twist.linear.y, 
      twist_msg.twist.linear.z
    );

    if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
      control_ref_.yaw[1] = twist_msg.twist.angular.z;
    }

    flags_.ref_received = true;
    return;
  };

  void DFPlugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg)
  {
    if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY)
    {
      control_ref_.pos = Vector3d(
        traj_msg.positions[0], 
        traj_msg.positions[1], 
        traj_msg.positions[2]
      );

      control_ref_.vel = Vector3d(
        traj_msg.velocities[0], 
        traj_msg.velocities[1], 
        traj_msg.velocities[2]
      );

      control_ref_.acc = Vector3d(
        traj_msg.accelerations[0], 
        traj_msg.accelerations[1], 
        traj_msg.accelerations[2]
      );

      control_ref_.yaw = Vector3d(
        traj_msg.positions[4], 
        traj_msg.velocities[4], 
        traj_msg.accelerations[4]
      );
    }
    else if (
      control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
      control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE
    ) {
      //control_ref_.yaw[0] = traj_msg.positions[4];
      control_ref_.yaw[0] = 0.0;
    }

    flags_.ref_received = true;
    return;
  };

  void DFPlugin::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                                   geometry_msgs::msg::TwistStamped &twist,
                                   as2_msgs::msg::Thrust &thrust)
  {

    RCLCPP_INFO(node_ptr_->get_logger(), "Control mode: %d", control_mode_in_.control_mode);


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
      return;
      //computeHOVER(pose, twist, thrust);
    }
    else
    {
      computeActions(pose, twist, thrust);
    }

    static rclcpp::Time last_time_ = node_ptr_->now();
    return;
  };

  bool DFPlugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                         const as2_msgs::msg::ControlMode &out_mode)
  {
    control_mode_in_ = in_mode;
    control_mode_out_ = out_mode;

    flags_.ref_received = false;
    flags_.state_received = false;

    controller_handler_->resetError();
    resetReferences();
    // resetCommands(); TODO: Enable when ControllerBase change getOutput to bool

    last_time_ = node_ptr_->now();
    
    return true;
  };

  rcl_interfaces::msg::SetParametersResult DFPlugin::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (auto &param : parameters)
    {
      // RCLCPP_INFO(node_ptr_->get_logger(), "Reading parameter %s with value: %f", param.get_name().c_str(),
      //             param.get_value<double>());
      if (controller_handler_->isParameter(param.get_name()))
      {
        // RCLCPP_INFO(node_ptr_->get_logger(), "Setting parameter %s with value: %f", param.get_name().c_str(),
        //             param.get_value<double>());
        controller_handler_->setParameter(param.get_name(), param.get_value<double>());

        // Remove the parameter from the list of parameters to be read
        parameters_to_read_.erase(
          std::remove(
            parameters_to_read_.begin(), 
            parameters_to_read_.end(), 
            param.get_name()
          ), 
          parameters_to_read_.end()
        );
        if (parameters_to_read_.empty())
        {
          flags_.parameters_read = true;
        }
      }
      else
      {
        RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not defined in controller params", param.get_name().c_str());
        result.successful = false;
        result.reason = "parameter not found";
      }
    }
    return result;
  };

  void DFPlugin::declareParameters()
  {
    std::vector<std::pair<std::string, double>> params = controller_handler_->getParametersList();
    for (auto &param : params)
    {
      node_ptr_->declare_parameter(param.first, param.second);
    }

    return;
  };

  void DFPlugin::computeActions(geometry_msgs::msg::PoseStamped &pose,
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
      computeHOVER(pose, twist, thrust);
      return;
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      f_des_ = controller_handler_->computeVelocityControl(
        uav_state_,
        control_ref_,
        dt
      );
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      f_des_ = controller_handler_->computeTrajectoryControl(
        uav_state_,
        control_ref_,
        dt
      );
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
        uav_state_,
        control_ref_.yaw[0],
        f_des_,
        // Output
        acro_, 
        thrust_
      );
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      controller_handler_->computeYawSpeedControl(
        // Input
        uav_state_,
        control_ref_.yaw[1],
        f_des_,
        dt,
        // Output
        acro_, 
        thrust_
      );
      break;
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

  void DFPlugin::computeHOVER(geometry_msgs::msg::PoseStamped &pose,
                              geometry_msgs::msg::TwistStamped &twist,
                              as2_msgs::msg::Thrust &thrust
  ) {
    rclcpp::Time current_time = node_ptr_->now();
    double dt = (current_time - last_time_).nanoseconds() / 1.0e9;
    last_time_ = current_time;

    resetCommands();
    f_des_ = controller_handler_->computeTrajectoryControl(
      uav_state_,
      control_ref_,
      dt
    );
    
    controller_handler_->computeYawAngleControl(
      // Input
      uav_state_,
      control_ref_.yaw[0],
      f_des_,
      // Output
      acro_, 
      thrust_
    );
    
    getOutput(pose, twist, thrust);
    return;
  };

  void DFPlugin::getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                          geometry_msgs::msg::TwistStamped &twist_msg,
                          as2_msgs::msg::Thrust &thrust_msg
  ) {
    twist_msg.header.stamp = node_ptr_->now();

    twist_msg.twist.angular.x = acro_(0);
    twist_msg.twist.angular.y = acro_(1);
    twist_msg.twist.angular.z = acro_(2);

    thrust_msg.header.stamp = node_ptr_->now();

    thrust_msg.thrust = thrust_;

    return;
  };

  void DFPlugin::resetState()
  {
    uav_state_.pos = Vector3d::Zero();
    uav_state_.vel = Vector3d::Zero();
    uav_state_.rot = Eigen::Matrix3d::Identity();
    return;
  };

  void DFPlugin::resetReferences()
  {
    control_ref_.pos = uav_state_.pos;
    control_ref_.vel = Vector3d::Zero();
    control_ref_.acc = Vector3d::Zero();

    Vector3d rot = uav_state_.rot.eulerAngles(0, 1, 2);

    control_ref_.yaw = Vector3d(
      rot[2], 
      0.0f, 
      0.0f
    );

    return;
  };

  void DFPlugin::resetCommands()
  {
    f_des_.setZero();
    acro_.setZero();
    thrust_ = 0.0f;
    return;
  };

} // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_differential_flatness::DFPlugin,
                       controller_plugin_base::ControllerBase)

#pragma endregion