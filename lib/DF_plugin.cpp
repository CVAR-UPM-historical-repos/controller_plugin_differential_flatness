#include "DF_plugin.hpp"

#pragma region DF_controller

namespace differential_flatness_controller
{
  DFController::DFController(const UAV_state& uav_state)
  {
    set_UAV_State(uav_state);

    Control_ref control_ref;
    control_ref.pos = uav_state.pos;
    control_ref.vel.setZero();
    control_ref.acc.setZero();
    control_ref.yaw.setZero();

    reset_error();
  };

  void DFController::set_UAV_State(const UAV_state& uav_state)
  {
    state_ = uav_state;
    return;
  };

  void DFController::set_references(const Control_ref& control_ref)
  {
    ref_ = control_ref;
    return;
  };

  bool DFController::set_parameter(const std::string& param, const double& value)
  {
    if (parameters_.count(param) == 1)
    {
      parameters_[param] = value;
      update_gains_();
      return true;
    }
    return false;
  };

  bool DFController::get_parameter(const std::string& param, double& value)
  {
    if (parameters_.count(param) == 1)
    {
      value = parameters_[param];
      return true;
    }
    return false;
  };

  bool DFController::set_Parameter_list(const std::vector<std::pair<std::string, double>> parameter_list)
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
    update_gains_();
    return true;
  };

  std::vector<std::pair<std::string,double>> DFController::get_parameters_list()
  {
    std::vector<std::pair<std::string,double>> list;
    for (auto& param : parameters_)
    {
      list.push_back({param.first, param.second});
    }
    return list;
  }; 

  Eigen::Vector3d DFController::get_position_error()
  {
    return position_accum_error_;
  };

  Eigen::Vector3d DFController::get_velocity_error()
  {
    return position_accum_error_;
  };

  void DFController::reset_error()
  {
    position_accum_error_.setZero();
    velocity_accum_error_.setZero();
    return;
  };
  
  // Compute the velocity control by velocity references
  Vector3d DFController::compute_velocity_control(const double& dt)
  {
    // Compute the proportional contribution (velocity)
    Vector3d velocity_error = ref_.vel - state_.vel;
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


  Vector3d DFController::compute_trajectory_control(const double& dt)
  {
    // Compute the proportional contribution (position) 
    Vector3d position_error = ref_.pos - state_.pos;
    Vector3d p_error_contribution = traj_Kp_lin_mat_ * position_error;

    // Compute de derivative error contribution (velocity)
    Vector3d velocity_error = ref_.vel - state_.vel;
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
    Vector3d force_acceleration = mass_ * ref_.acc;

    // Compute gravity compensation
    Vector3d force_gravity = mass_ * gravitational_accel_;

    // Return desired force with the gravity compensation
    Vector3d desired_force = - force_error + force_acceleration + force_gravity;
    return desired_force;
  }

  void DFController::update_gains_()
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
    flags_.ref_generated = false;
    flags_.hover_position = false;
    flags_.state_received = false;
    flags_.parameters_read = false;

    UAV_state uav_state;
    uav_state.pos = Vector3d::Zero();
    uav_state.rot = Vector3d::Zero();
    uav_state.vel = Vector3d::Zero();

    controller_handler_ = std::make_shared<DFController>(uav_state);
  };

  void DFPlugin::updateState(const nav_msgs::msg::Odometry &odom)
  {
    return;
  };

  void DFPlugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    return;
  };

  void DFPlugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg)
  {
    return;
  };

  void DFPlugin::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                                   geometry_msgs::msg::TwistStamped &twist,
                                   as2_msgs::msg::Thrust &thrust)
  {
    return;
  };

  bool DFPlugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                             const as2_msgs::msg::ControlMode &out_mode)
  {
    return true;
  };

  rcl_interfaces::msg::SetParametersResult DFPlugin::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  };

  void DFPlugin::set_default_parameters()
  {
    return;
  };

  void DFPlugin::declareParameters(std::unordered_map<std::string, double> &params)
  {
    return;
  };

  void DFPlugin::computeActions(geometry_msgs::msg::PoseStamped &pose,
                                    geometry_msgs::msg::TwistStamped &twist,
                                    as2_msgs::msg::Thrust &thrust)
  {
    return;
  };

  void DFPlugin::computeHOVER(geometry_msgs::msg::PoseStamped &pose,
                                  geometry_msgs::msg::TwistStamped &twist,
                                  as2_msgs::msg::Thrust &thrust)
  {
    return;
  };

  void DFPlugin::getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                               geometry_msgs::msg::TwistStamped &twist_msg,
                               as2_msgs::msg::Thrust &thrust_msg)
  {
    return;
  };

  void DFPlugin::resetState()
  {
    return;
  };

  void DFPlugin::initialize_references()
  {
    return;
  };

  void DFPlugin::reset_references()
  {
    return;
  };

  void DFPlugin::reset_commands()
  {
    return;
  };

} // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_differential_flatness::DFPlugin,
                       controller_plugin_base::ControllerBase)

#pragma endregion