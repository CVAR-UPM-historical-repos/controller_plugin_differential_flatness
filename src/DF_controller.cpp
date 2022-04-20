/*!********************************************************************************
 * \brief     Differential Flatness controller Implementation
 * \authors   Miguel Fernandez-Cortizas
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "DF_controller.hpp"
#include <rcl_interfaces/msg/detail/floating_point_range__struct.hpp>

PD_controller::PD_controller() : as2::Node("differential_flatness_controller", rclcpp::NodeOptions()
                                                                                   .allow_undeclared_parameters(true)
                                                                                   .automatically_declare_parameters_from_overrides(true)
                                                                                   .start_parameter_services(true))
{
  RCLCPP_INFO(this->get_logger(),
              "Parameter blackboard node named '%s' ready, and serving '%zu' parameters already!",
              this->get_name(), this->list_parameters({this->get_namespace()}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names.size());

  for (auto &parameter_name : this->list_parameters(
                                      {}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)
                                  .names)
  {
    // RCLCPP_INFO(this->get_logger(), "Parameter: %s", parameter_name.c_str());
    if (this->get_parameter(parameter_name).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      parameters_.emplace(parameter_name, this->get_parameter(parameter_name).as_double());
    }
  }

  static auto callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PD_controller::parametersCallback, this, std::placeholders::_1));

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->generate_global_name(as2_names::topics::self_localization::odom),
      as2_names::topics::self_localization::qos,
      std::bind(&PD_controller::CallbackOdomTopic, this, std::placeholders::_1));
  
  sub_traj_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      this->generate_global_name(as2_names::topics::motion_reference::trajectory),
      as2_names::topics::motion_reference::qos,
      std::bind(&PD_controller::CallbackTrajTopic, this, std::placeholders::_1));

  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_global_name(as2_names::topics::motion_reference::twist),
      as2_names::topics::motion_reference::qos,
      std::bind(&PD_controller::CallbackTwistTopic, this, std::placeholders::_1));

  set_control_mode_srv_ = this->create_service<as2_msgs::srv::SetControllerControlMode>(
      this->generate_global_name(as2_names::services::motion_reference::setcontrolmode),
      std::bind(
          &PD_controller::setControlModeSrvCall, this,
          std::placeholders::_1, // Corresponds to the 'request'  input
          std::placeholders::_2  // Corresponds to the 'response' input
          ));
}

void PD_controller::setup()
{

  update_gains(parameters_);

  RCLCPP_INFO(this->get_logger(), "PD controller non-linearized");
  RCLCPP_INFO(this->get_logger(), "uav_mass = %f", mass);

  flags_.ref_generated = false;
  flags_.hover_position = false;
  flags_.state_received = false;

  // set all refs to zefs
  for (auto dof : refs_)
  {
    for (auto elem : dof)
      elem = 0.0f;
  }
}

void PD_controller::run()
{
  if (control_mode_ == ControlMode::UNSET)
  {
    RCLCPP_WARN(this->get_logger(), "Control mode not set");
    return;
  }

  if (!flags_.state_received)
  {
    RCLCPP_WARN(this->get_logger(), "State not received yet");
    return;
  }

  if (!flags_.ref_generated)
  {
    RCLCPP_WARN(this->get_logger(), "State changed, but ref not recived yet");
    // PD_controller::reset_references();
    computeActions(ControlMode::HOVER);
  }
  else 
  {
    computeActions(control_mode_);
  }
  publishActions();
};

void PD_controller::update_gains(const std::unordered_map<std::string, double> &params)
{
  // for (auto it = params.begin(); it != params.end(); it++) {
  //   RCLCPP_INFO(this->get_logger(), "Updating gains: %s = %f", it->first.c_str(), it->second);
  // }

  Eigen::Vector3d speed_Kp_lin(params.at("speed_following.speed_Kp.x"),
                               params.at("speed_following.speed_Kp.y"),
                               params.at("speed_following.speed_Kp.z"));
  Eigen::Vector3d speed_Kd_lin(params.at("speed_following.speed_Kd.x"),
                               params.at("speed_following.speed_Kd.y"),
                               params.at("speed_following.speed_Kd.z"));
  Eigen::Vector3d speed_Ki_lin(params.at("speed_following.speed_Ki.x"),
                               params.at("speed_following.speed_Ki.y"),
                               params.at("speed_following.speed_Ki.z"));

  Eigen::Vector3d traj_Kp_lin(params.at("trajectory_following.position_Kp.x"),
                              params.at("trajectory_following.position_Kp.y"),
                              params.at("trajectory_following.position_Kp.z"));
  Eigen::Vector3d traj_Kd_lin(params.at("trajectory_following.position_Kd.x"),
                              params.at("trajectory_following.position_Kd.y"),
                              params.at("trajectory_following.position_Kd.z"));
  Eigen::Vector3d traj_Ki_lin(params.at("trajectory_following.position_Ki.x"),
                              params.at("trajectory_following.position_Ki.y"),
                              params.at("trajectory_following.position_Ki.z"));

  Eigen::Vector3d Kp_ang(params.at("angular_speed_controller.angular_gain.x"),
                         params.at("angular_speed_controller.angular_gain.y"),
                         params.at("angular_speed_controller.angular_gain.z"));

  traj_Kp_lin_mat = traj_Kp_lin.asDiagonal();
  traj_Kd_lin_mat = traj_Kd_lin.asDiagonal();
  traj_Ki_lin_mat = traj_Ki_lin.asDiagonal();

  speed_Kp_lin_mat = speed_Kp_lin.asDiagonal();
  speed_Kd_lin_mat = speed_Kd_lin.asDiagonal();
  speed_Ki_lin_mat = speed_Ki_lin.asDiagonal();

  Kp_ang_mat = Kp_ang.asDiagonal();

  mass = params.at("uav_mass");
  antiwindup_cte_ = params.at("antiwindup_cte");
};

Vector3d PD_controller::computeForceDesiredByTraj()
{
  static Eigen::Vector3d gravitational_accel(0, 0, g);

  Vector3d r(state_.pos[0], state_.pos[1], state_.pos[2]);
  Vector3d rdot(state_.vel[0], state_.vel[1], state_.vel[2]);
  Vector3d r_t(refs_[0][0], refs_[1][0], refs_[2][0]);
  Vector3d rdot_t(refs_[0][1], refs_[1][1], refs_[2][1]);
  Vector3d rddot_t(refs_[0][2], refs_[1][2], refs_[2][2]);

  Vector3d e_p = r - r_t;
  Vector3d e_v = rdot - rdot_t;

  accum_error_ += e_p;

  for (short j = 0; j < 3; j++)
  {
    float antiwindup_value = antiwindup_cte_ / traj_Ki_lin_mat.diagonal()[j];
    accum_error_[j] = (accum_error_[j] > antiwindup_value) ? antiwindup_value : accum_error_[j];
    accum_error_[j] = (accum_error_[j] < -antiwindup_value) ? -antiwindup_value : accum_error_[j];
  }

  Vector3d F_des = -traj_Kp_lin_mat * e_p - traj_Ki_lin_mat * accum_error_ - traj_Kd_lin_mat * e_v +
                   mass * gravitational_accel + mass * rddot_t;
  return F_des;
}

Vector3d PD_controller::computeForceDesiredBySpeed()
{
  const static Eigen::Vector3d gravitational_accel(0, 0, g);

  Vector3d rdot(state_.vel[0], state_.vel[1], state_.vel[2]);
  Vector3d rdot_t(refs_[0][1], refs_[1][1], refs_[2][1]);

  Vector3d vel_error_contribution;
  Vector3d dvel_error_contribution;
  Vector3d accum_vel_error_contribution;

  // compute vel error contribution
  Vector3d e_v = rdot_t - rdot;
  vel_error_contribution = speed_Kp_lin_mat * e_v;

  // compute dt
  static rclcpp::Time last_time = this->now();
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time).nanoseconds() / 1.0e9;
  last_time = current_time;

  static Vector3d last_e_v = e_v;
  const double alpha = 0.1;
  static Vector3d filtered_d_e_v = e_v;

  Vector3d inc_e_v = (e_v - last_e_v);
  filtered_d_e_v = alpha * inc_e_v + (1 - alpha) * filtered_d_e_v;
  last_e_v = e_v;

  // compute dvel error contribution
  dvel_error_contribution = speed_Kd_lin_mat * filtered_d_e_v / dt;

  // compute accum_error
  accum_error_ += e_v * dt;

  // compute antiwindup
  for (short j = 0; j < 3; j++)
  {
    float antiwindup_value = antiwindup_cte_ / speed_Ki_lin_mat.diagonal()[j];
    accum_error_[j] = (accum_error_[j] > antiwindup_value) ? antiwindup_value : accum_error_[j];
    accum_error_[j] = (accum_error_[j] < -antiwindup_value) ? -antiwindup_value : accum_error_[j];
  }

  // compute accum_vel_error_contribution
  accum_vel_error_contribution = speed_Ki_lin_mat * accum_error_;

  // compute a_des
  Vector3d a_des = vel_error_contribution + dvel_error_contribution + accum_vel_error_contribution;

  // compute F_des
  Vector3d F_des = mass * a_des + mass * gravitational_accel;

  return F_des;
}

void PD_controller::reset_references() {

  RCLCPP_INFO(this->get_logger(), "Resetting references");

  // Set errors to zero
  accum_error_.setZero();

  // Set reference to current state
  refs_[0][0] = state_.pos[0];
  refs_[1][0] = state_.pos[1];
  refs_[2][0] = state_.pos[2];

  refs_[3][0] = state_.rot[2];

  refs_[0][1] = 0.0f;
  refs_[1][1] = 0.0f;
  refs_[2][1] = 0.0f;
  refs_[3][1] = 0.0f;

  refs_[0][2] = 0.0f;
  refs_[1][2] = 0.0f;
  refs_[2][2] = 0.0f;
  refs_[3][2] = 0.0f;
}

void PD_controller::computeActions(uint8_t control_mode)
{
  Vector3d F_des;
  switch (control_mode)
  {  
  case ControlMode::HOVER:
    RCLCPP_INFO(this->get_logger(), "HOVERING");
    RCLCPP_INFO(this->get_logger(), "refs_[0][0] = %f", refs_[0][0]);
    RCLCPP_INFO(this->get_logger(), "refs_[1][0] = %f", refs_[1][0]);
    RCLCPP_INFO(this->get_logger(), "refs_[2][0] = %f", refs_[2][0]);
    RCLCPP_INFO(this->get_logger(), "refs_[3][0] = %f", refs_[3][0]);
    F_des = PD_controller::computeForceDesiredByTraj();
    break;

  case ControlMode::TRAJECTORY:
    F_des = PD_controller::computeForceDesiredByTraj();
    break;

  case ControlMode::SPEED:
    F_des = PD_controller::computeForceDesiredBySpeed();
    break;

  case ControlMode::UNSET:
    RCLCPP_WARN(this->get_logger(), "2: Trajectory not generated or control mode not set");
    return;
    break;

  default:
    throw std::runtime_error("Unknown control mode");
    break;
  }

  Vector3d zb_des = F_des.normalized();

  Vector3d xc_des(cos(refs_[3][0]), sin(refs_[3][0]), 0);

  // Vector3d xc_des(cos(state_.rot[2]),sin(state_.rot[2]),0);
  Vector3d yb_des = zb_des.cross(xc_des).normalized();
  Vector3d xb_des = yb_des.cross(zb_des).normalized();

  Eigen::Matrix3d R_des;
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  Eigen::Matrix3d Mat_e_rot = (R_des.transpose() * Rot_matrix - Rot_matrix.transpose() * R_des);

  Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
  Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

  u1 = (float)F_des.dot(Rot_matrix.col(2).normalized());

  Vector3d outputs = -Kp_ang_mat * E_rot;

  u2[0] = outputs(0); // ROLL
  u2[1] = outputs(1); // PITCH
  u2[2] = outputs(2); // YAW
};

void PD_controller::publishActions()
{
  static as2::controlCommandsHandlers::AcroControl acro_controller(this);
  acro_controller.sendAngleRatesWithThrust(u2[0], u2[1], u2[2], u1);
};

bool PD_controller::setControlMode(const as2_msgs::msg::ControllerControlMode &msg)
{
  RCLCPP_INFO(this->get_logger(), "Setting controller control mode");
  PD_controller::reset_references();

  flags_.ref_generated = false;

  switch (msg.control_mode)
  {
  case as2_msgs::msg::ControllerControlMode::HOVER:
  {
    control_mode_ = ControlMode::HOVER;
    RCLCPP_INFO(this->get_logger(), "SPEED_MODE ENABLED");
  }
  break;
  case as2_msgs::msg::ControllerControlMode::TRAJECTORY:
  {
    control_mode_ = ControlMode::TRAJECTORY;
    RCLCPP_INFO(this->get_logger(), "POSITION_MODE ENABLED");
  }
  break;
  case as2_msgs::msg::ControllerControlMode::SPEED:
  {
    control_mode_ = ControlMode::SPEED;
    RCLCPP_INFO(this->get_logger(), "SPEED_MODE ENABLED");
  }
  break;
  default:
    RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
    control_mode_ = ControlMode::UNSET;
    return false;
  }
  return true;
};

/* --------------------------- CALLBACKS ---------------------------*/
rcl_interfaces::msg::SetParametersResult PD_controller::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (auto &param : parameters)
  {
    // check if the parameter is defined in parameters_
    if (parameters_.find(param.get_name()) != parameters_.end())
    {
      parameters_[param.get_name()] = param.get_value<double>();
    }
    else
    {
      result.successful = false;
      result.reason = "parameter not found";
    }
  }
  if (result.successful)
  {
    update_gains(parameters_);
  }
  return result;
}

void PD_controller::CallbackTrajTopic(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  if (control_mode_ != ControlMode::TRAJECTORY)
  {
    return;
  }
  auto &traj_msg = *(msg.get());
  flags_.ref_generated = true;

  for (int i = 0; i < 4; i++)
  {
    refs_[i][0] = traj_msg.positions[i];
    refs_[i][1] = traj_msg.velocities[i];
    refs_[i][2] = traj_msg.accelerations[i];
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory received");

  /*
  Matrix:
  | x_ref_x | v_ref_x | a_ref_x |
  | x_ref_y | v_ref_y | a_ref_y |
  | x_ref_z | v_ref_z | a_ref_z |
  | x_ref_y | v_ref_y | a_ref_y |
  */
}

void PD_controller::CallbackTwistTopic(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (control_mode_ != ControlMode::SPEED)
  {
    return;
  }
  auto &twist_msg = *(msg.get());
  flags_.ref_generated = true;

  refs_[0][1] = twist_msg.twist.linear.x;
  refs_[1][1] = twist_msg.twist.linear.y;
  refs_[2][1] = twist_msg.twist.linear.z;
  refs_[3][1] = twist_msg.twist.angular.z;
  
  // Yaw
  refs_[3][0] = twist_msg.twist.angular.y;

  // For position and speed, reset the reference to actual state
  // for (int i = 0; i < 3; i++)
  // {
  //   refs_[i][0] = state_.pos[i];
  //   refs_[i][2] = 0.0f;
  // }

  RCLCPP_INFO(this->get_logger(), "Twist received");
}

void PD_controller::CallbackOdomTopic(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  state_.pos[0] = msg->pose.pose.position.x;
  state_.pos[1] = msg->pose.pose.position.y;
  state_.pos[2] = msg->pose.pose.position.z;

  state_.vel[0] = msg->twist.twist.linear.x;
  state_.vel[1] = msg->twist.twist.linear.y;
  state_.vel[2] = msg->twist.twist.linear.z;

  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

  Rot_matrix = q.toRotationMatrix();
  state_.rot = Rot_matrix.eulerAngles(0, 1, 2);

  flags_.state_received = true;
}

void PD_controller::setControlModeSrvCall(
    const std::shared_ptr<as2_msgs::srv::SetControllerControlMode::Request> request,
    std::shared_ptr<as2_msgs::srv::SetControllerControlMode::Response> response)
{
  bool success = this->setControlMode(request->control_mode);
  response->success = success;
  if (!success)
  {
    RCLCPP_ERROR(this->get_logger(), "ERROR: UNABLE TO SET THIS CONTROL MODE TO THIS PLATFORM");
  }
};
