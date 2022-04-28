/*!********************************************************************************
 * \brief     Differential Flatness controller Implementation
 * \authors   Miguel Fernandez-Cortizas
 :on
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

namespace controller_plugin_differential_flatness
{

  void PDController::ownInitialize()
  {
    // TODO: Read params
    get_default_parameters();
    // readParameters(param_names_);
    // update_gains(parameters_);

    // Free parameters name vector
    param_names_ = std::vector<std::string>();

    resetState();
    initialize_references();
    resetErrors();
    resetCommands();

    last_time_ = node_ptr_->now();

    return;
  };

  void PDController::updateState(const nav_msgs::msg::Odometry &odom)
  {
    // RCLCPP_WARN(node_ptr_->get_logger(), "Updating state");
    state_.pos[0] = odom.pose.pose.position.x;
    state_.pos[1] = odom.pose.pose.position.y;
    state_.pos[2] = odom.pose.pose.position.z;

    state_.vel[0] = odom.twist.twist.linear.x;
    state_.vel[1] = odom.twist.twist.linear.y;
    state_.vel[2] = odom.twist.twist.linear.z;

    Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

    Rot_matrix = q.toRotationMatrix();
    state_.rot = Rot_matrix.eulerAngles(0, 1, 2);

    // RCLCPP_WARN(node_ptr_->get_logger(), "Rot_matrix (0,:): %f %f %f", Rot_matrix(0, 0), Rot_matrix(0, 1),
    //             Rot_matrix(0, 2));

    // RCLCPP_WARN(node_ptr_->get_logger(), "Rot_matrix (1,:): %f %f %f", Rot_matrix(1, 0), Rot_matrix(1, 1),
    //             Rot_matrix(1, 2));
    
    // RCLCPP_WARN(node_ptr_->get_logger(), "Rot_matrix (2,:): %f %f %f", Rot_matrix(2, 0), Rot_matrix(2, 1),
    //             Rot_matrix(2, 2));

    // RCLCPP_WARN(node_ptr_->get_logger(), "State updated");

    flags_.state_received = true;

    return;
  };

  void PDController::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED)
    {
      return;
    }

    flags_.ref_generated = true;

    refs_[0][1] = twist_msg.twist.linear.x;
    refs_[1][1] = twist_msg.twist.linear.y;
    refs_[2][1] = twist_msg.twist.linear.z;
    refs_[3][1] = twist_msg.twist.angular.z;

    // Yaw
    refs_[3][0] = twist_msg.twist.angular.y;

    return;
  };

  void PDController::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY)
    {
      return;
    }

    flags_.ref_generated = true;

    for (int i = 0; i < 4; i++)
    {
      refs_[i][0] = traj_msg.positions[i];
      refs_[i][1] = traj_msg.velocities[i];
      refs_[i][2] = traj_msg.accelerations[i];
    }

    return;

    //   Matrix:
    //   | x_ref_x   | v_ref_x   | a_ref_x   |
    //   | x_ref_y   | v_ref_y   | a_ref_y   |
    //   | x_ref_z   | v_ref_z   | a_ref_z   |
    //   | x_ref_yaw | v_ref_yaw | a_ref_yaw |
  };

  void PDController::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                                   geometry_msgs::msg::TwistStamped &twist,
                                   as2_msgs::msg::Thrust &thrust)
  {
    if (!flags_.state_received)
    {
      RCLCPP_WARN_ONCE(node_ptr_->get_logger(), "State not received yet");
      return;
    }

    if (!flags_.ref_generated)
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "State changed, but ref not recived yet");
      computeHOVER(pose, twist, thrust);
    }
    else
    {
      computeActions(pose, twist, thrust);
    }

    return;
  };

  bool PDController::setMode(const as2_msgs::msg::ControlMode &in_mode,
                             const as2_msgs::msg::ControlMode &out_mode)
  {

    // TODO: Check if assignment is valid
    // control_mode_in_ = in_mode;
    // control_mode_out_ = out_mode;

    control_mode_in_.yaw_mode = in_mode.yaw_mode;
    control_mode_in_.control_mode = in_mode.control_mode;
    control_mode_in_.reference_frame = in_mode.reference_frame;

    control_mode_out_.yaw_mode = out_mode.yaw_mode;
    control_mode_out_.control_mode = out_mode.control_mode;
    control_mode_out_.reference_frame = out_mode.reference_frame;

    last_time_ = node_ptr_->now();

    reset_references();
    resetErrors();
    return true;
  };

  void PDController::readParameters(std::vector<std::string> &params)
  {
    for (auto it = std::begin(params); it != std::end(params); ++it)
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Declaring parameter: %s", it->c_str());
      node_ptr_->declare_parameter(*it);
      parameters_.emplace(*it, 0.0);
      // RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s added to parameters_", it->c_str());
    }

    for (auto &parameter_name :
         node_ptr_->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)
             .names)
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Parameter: %s", parameter_name.c_str());

      // if (parameters_.count(parameter_name))
      // {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Parameter: %s in parameters_", parameter_name.c_str());

      //   if (node_ptr_->get_parameter(parameter_name).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      //   {
      //     RCLCPP_INFO(node_ptr_->get_logger(), "Parameter: %s is double", parameter_name.c_str());
      //   }
      //   else
      //   {
      //     RCLCPP_INFO(node_ptr_->get_logger(), "Parameter: %s is not double", parameter_name.c_str());
      //   }
      // }
      // else
      // {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Parameter: %s not in parameters_", parameter_name.c_str());
      // }

      if (parameters_.count(parameter_name) && node_ptr_->get_parameter(parameter_name).get_type() ==
                                                   rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s modify to parameters_", parameter_name.c_str());
        parameters_[parameter_name] = node_ptr_->get_parameter(parameter_name).as_double();
        // parameters_.emplace(parameter_name, node_ptr_->get_parameter(parameter_name).as_double());
      }
      else
      {
        RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s not modify to parameters_", parameter_name.c_str());
      }
    }

    return;
  }

  void PDController::get_default_parameters()
  {
    mass = 1.5;
    antiwindup_cte_ = 1.0;

    Eigen::Vector3d speed_Kp_lin(3.0,
                                 3.0,
                                 4.0);
    Eigen::Vector3d speed_Kd_lin(0.0,
                                 0.0,
                                 0.0);
    Eigen::Vector3d speed_Ki_lin(0.0,
                                 0.0,
                                 0.01);

    Eigen::Vector3d traj_Kp_lin(6.0,
                                6.0,
                                6.0);
    Eigen::Vector3d traj_Kd_lin(0.01,
                                0.01,
                                0.01);
    Eigen::Vector3d traj_Ki_lin(3.0,
                                3.0,
                                3.0);

    Eigen::Vector3d Kp_ang(5.5,
                           5.5,
                           5.0);

    traj_Kp_lin_mat = traj_Kp_lin.asDiagonal();
    traj_Kd_lin_mat = traj_Kd_lin.asDiagonal();
    traj_Ki_lin_mat = traj_Ki_lin.asDiagonal();

    speed_Kp_lin_mat = speed_Kp_lin.asDiagonal();
    speed_Kd_lin_mat = speed_Kd_lin.asDiagonal();
    speed_Ki_lin_mat = speed_Ki_lin.asDiagonal();

    Kp_ang_mat = Kp_ang.asDiagonal();

    return;
  }

  void PDController::update_gains(const std::unordered_map<std::string, double> &params)
  {
    // for (auto it = params.begin(); it != params.end(); it++) {
    //   RCLCPP_INFO(this->get_logger(), "Updating gains: %s = %f", it->first.c_str(), it->second);
    // }

    mass = params.at("uav_mass");
    antiwindup_cte_ = params.at("antiwindup_cte");

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

    return;
  };

  void PDController::resetState()
  {
    state_.pos[0] = 0.0;
    state_.pos[1] = 0.0;
    state_.pos[2] = 0.0;
    state_.vel[0] = 0.0;
    state_.vel[1] = 0.0;
    state_.vel[2] = 0.0;
    state_.rot[0] = 0.0;
    state_.rot[1] = 0.0;
    state_.rot[2] = 0.0;
  }

  void PDController::initialize_references()
  {
    // set all refs to zefs
    for (auto dof : refs_)
    {
      for (auto elem : dof)
        elem = 0.0f;
    }

    return;
  };

  void PDController::reset_references()
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Resetting references");

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

    return;
  };

  void PDController::resetErrors()
  {
    // Set errors to zero
    accum_error_.setZero();
    return;
  };

  void PDController::computeActions(geometry_msgs::msg::PoseStamped &pose,
                                    geometry_msgs::msg::TwistStamped &twist,
                                    as2_msgs::msg::Thrust &thrust)
  {
    resetCommands();

    // RCLCPP_INFO(node_ptr_->get_logger(), "Computing PD controller actions");

    // // state_
    // RCLCPP_INFO(node_ptr_->get_logger(), "State: %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //             state_.pos[0],
    //             state_.pos[1],
    //             state_.pos[2],
    //             state_.vel[0],
    //             state_.vel[1],
    //             state_.vel[2],
    //             state_.rot[0],
    //             state_.rot[1],
    //             state_.rot[2]);

    // // ref_
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[0][0] = %f", refs_[0][0]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[1][0] = %f", refs_[1][0]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[2][0] = %f", refs_[2][0]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[3][0] = %f", refs_[3][0]);

    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[0][1] = %f", refs_[0][1]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[1][1] = %f", refs_[1][1]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[2][1] = %f", refs_[2][1]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[3][1] = %f", refs_[3][1]);

    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[0][2] = %f", refs_[0][2]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[1][2] = %f", refs_[1][2]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[2][2] = %f", refs_[2][2]);
    // RCLCPP_INFO(node_ptr_->get_logger(), "refs_[3][2] = %f", refs_[3][2]);

    switch (control_mode_in_.control_mode)
    {
    case as2_msgs::msg::ControlMode::HOVER:
      computeHOVER(pose, twist, thrust);
      return;
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      f_des_ = computeSpeedControl();
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      f_des_ = computeTrajectoryControl();
      break;
    default:
      RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Unknown control mode");
      return;
      break;
    }

    // RCLCPP_INFO(node_ptr_->get_logger(), "f_des_ = %f, %f, %f", f_des_(0), f_des_(1), f_des_(2));

    switch (control_mode_in_.yaw_mode)
    {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      computeYawAngleControl(acro_, thrust_);
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      computeYawSpeedControl(acro_, thrust_);
      break;
    default:
      RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Unknown yaw mode");
      return;
      break;
    }

    // RCLCPP_INFO(node_ptr_->get_logger(), "thrust_ = %f", thrust_);
    // RCLCPP_INFO(node_ptr_->get_logger(), "acro_ = %f, %f, %f", acro_(0), acro_(1), acro_(2));

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

  void PDController::resetCommands()
  {
    f_des_.setZero();
    acro_.setZero();
    thrust_ = 0.0f;
  }

  void PDController::computeHOVER(geometry_msgs::msg::PoseStamped &pose,
                                  geometry_msgs::msg::TwistStamped &twist,
                                  as2_msgs::msg::Thrust &thrust)
  {
    resetCommands();
    f_des_ = computeTrajectoryControl();
    computeYawAngleControl(acro_, thrust_);
    getOutput(pose, twist, thrust);
    return;
  }

  Vector3d PDController::computeSpeedControl()
  {
    // RCLCPP_INFO(node_ptr_->get_logger(), "Computing speed control");
    // RCLCPP_INFO(node_ptr_->get_logger(), "speed_Kp_lin_mat: %f %f %f",
    //             speed_Kp_lin_mat(0, 0),
    //             speed_Kp_lin_mat(1, 1),
    //             speed_Kp_lin_mat(2, 2));

    Vector3d f_des = Vector3d::Zero();

    Vector3d rdot(state_.vel[0], state_.vel[1], state_.vel[2]);
    Vector3d rdot_t(refs_[0][1], refs_[1][1], refs_[2][1]);

    Vector3d vel_error_contribution;
    Vector3d dvel_error_contribution;
    Vector3d accum_vel_error_contribution;

    // compute vel error contribution
    Vector3d e_v = rdot_t - rdot;
    vel_error_contribution = speed_Kp_lin_mat * e_v;

    // compute dt
    static rclcpp::Time last_time = node_ptr_->now();
    rclcpp::Time current_time = node_ptr_->now();
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
    f_des = mass * a_des + mass * gravitational_accel;

    return f_des;
  };

  Vector3d PDController::computeTrajectoryControl()
  {
    Vector3d f_des = Vector3d::Zero();

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

    f_des = -traj_Kp_lin_mat * e_p - traj_Ki_lin_mat * accum_error_ - traj_Kd_lin_mat * e_v +
            mass * gravitational_accel + mass * rddot_t;

    return f_des;
  };

  void PDController::computeYawAngleControl(Vector3d &acro, float &thrust)
  {
    // RCLCPP_INFO(node_ptr_->get_logger(), "computeYawAngleControl");

    Vector3d zb_des = f_des_.normalized();

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

    thrust = (float)f_des_.dot(Rot_matrix.col(2).normalized());

    Vector3d outputs = -Kp_ang_mat * E_rot;

    acro(0) = outputs(0); // ROLL
    acro(1) = outputs(1); // PITCH
    acro(2) = outputs(2); // YAW

    return;
  };

  void PDController::computeYawSpeedControl(Vector3d &acro, float &thrust)
  {
    // RCLCPP_INFO(node_ptr_->get_logger(), "Yaw Speed Control");

    Vector3d zb_des = f_des_.normalized();

    float yaw_state = state_.rot[2];
    float yawdot_ref = refs_[3][1];
    rclcpp::Time current_time = node_ptr_->now();

    double dt = (current_time - last_time_).nanoseconds() / 1.0e9;
    last_time_ = current_time;

    float yaw_des = yaw_state + yawdot_ref * dt;

    Vector3d xc_des(cos(yaw_des), sin(yaw_des), 0);

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

    // RCLCPP_INFO(node_ptr_->get_logger(), "Rot_matrix (0,:): %f %f %f", Rot_matrix(0, 0), Rot_matrix(0, 1),
    //             Rot_matrix(0, 2));

    // RCLCPP_INFO(node_ptr_->get_logger(), "Rot_matrix (1,:): %f %f %f", Rot_matrix(1, 0), Rot_matrix(1, 1),
    //             Rot_matrix(1, 2));
    
    // RCLCPP_INFO(node_ptr_->get_logger(), "Rot_matrix (2,:): %f %f %f", Rot_matrix(2, 0), Rot_matrix(2, 1),
    //             Rot_matrix(2, 2));

    // RCLCPP_INFO(node_ptr_->get_logger(), "Rot_matrix.col(2).normalized(): %f %f %f",
    //             Rot_matrix.col(2).normalized()(0), Rot_matrix.col(2).normalized()(1),
    //             Rot_matrix.col(2).normalized()(2));

    // RCLCPP_INFO(node_ptr_->get_logger(), "f_des_: %f", f_des_.norm());

    thrust = (float)f_des_.dot(Rot_matrix.col(2).normalized());

    Vector3d outputs = -Kp_ang_mat * E_rot;

    acro(0) = outputs(0); // ROLL
    acro(1) = outputs(1); // PITCH
    acro(2) = outputs(2); // YAW

    return;
  };

  void PDController::getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
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
  }
} // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_differential_flatness::PDController,
                       controller_plugin_base::ControllerBase)
