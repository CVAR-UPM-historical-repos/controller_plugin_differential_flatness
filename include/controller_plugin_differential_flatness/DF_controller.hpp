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

#ifndef __PD_CONTROLLER_H__
#define __PD_CONTROLLER_H__

// Std libraries
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <vector>

// Eigen

// #include "as2_control_command_handlers/acro_control.hpp"
#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "as2_msgs/srv/set_controller_control_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <Eigen/Dense>
#include "Eigen/src/Core/Matrix.h"

#include "controller_plugin_base/controller_base.hpp"

namespace controller_plugin_differential_flatness
{
  using Vector3d = Eigen::Vector3d;

  struct Control_flags
  {
    bool ref_generated;
    bool hover_position;
    bool state_received;
  };

  struct UAV_state
  {
    // State composed of s = [pose ,d_pose]'
    Vector3d pos;
    Vector3d rot;
    Vector3d vel;
  };

  class PDController : public controller_plugin_base::ControllerBase
  {
  public:
    PDController(){};
    ~PDController(){};

  private:
    as2::Node *node_ptr_;

  public:
    void initialize(as2::Node *node_ptr);

    void updateState(const nav_msgs::msg::Odometry &odom);

    void updateReference(const geometry_msgs::msg::PoseStamped &ref){};
    void updateReference(const geometry_msgs::msg::TwistStamped &ref);
    void updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &ref);
    void updateReference(const as2_msgs::msg::Thrust &ref){};

    void computeOutput(geometry_msgs::msg::PoseStamped &pose,
                       geometry_msgs::msg::TwistStamped &twist,
                       as2_msgs::msg::Thrust &thrust);

    bool setMode(const as2_msgs::msg::ControlMode &mode_in,
                 const as2_msgs::msg::ControlMode &mode_out);

    void update_gains(const std::unordered_map<std::string, double> &params);

  private:
    rclcpp::Time last_time_;

    as2_msgs::msg::ControlMode control_mode_in_;
    as2_msgs::msg::ControlMode control_mode_out_;

    UAV_state state_;
    Control_flags flags_;

    std::unordered_map<std::string, double> parameters_;

    float mass = 1.5f;
    const float g = 9.81;
    const Eigen::Vector3d gravitational_accel = Eigen::Vector3d(0, 0, g);

    Eigen::Vector3d accum_error_;

    Eigen::Matrix3d Kp_ang_mat;

    Eigen::Matrix3d traj_Kd_lin_mat;
    Eigen::Matrix3d traj_Kp_lin_mat;
    Eigen::Matrix3d traj_Ki_lin_mat;

    Eigen::Matrix3d speed_Kd_lin_mat;
    Eigen::Matrix3d speed_Kp_lin_mat;
    Eigen::Matrix3d speed_Ki_lin_mat;

    Eigen::Matrix3d Rot_matrix;
    float antiwindup_cte_ = 1.0f;

    float u1 = 0.0;
    float u2[3] = {0.0, 0.0, 0.0};

    Vector3d f_des_ = Vector3d::Zero();
    Vector3d acro_ = Vector3d::Zero();
    float thrust_ = 0.0;

    std::array<std::array<float, 3>, 4> refs_;

    // List of string names for the parameters
    std::vector<std::string> param_names_ = {
        "speed_following.speed_Kp.x",
        "speed_following.speed_Kp.y",
        "speed_following.speed_Kp.z",
        "speed_following.speed_Kd.x",
        "speed_following.speed_Kd.y",
        "speed_following.speed_Kd.z",
        "speed_following.speed_Ki.x",
        "speed_following.speed_Ki.y",
        "speed_following.speed_Ki.z",
        "trajectory_following.position_Kp.x",
        "trajectory_following.position_Kp.y",
        "trajectory_following.position_Kp.z",
        "trajectory_following.position_Kd.x",
        "trajectory_following.position_Kd.y",
        "trajectory_following.position_Kd.z",
        "trajectory_following.position_Ki.x",
        "trajectory_following.position_Ki.y",
        "trajectory_following.position_Ki.z",
        "angular_speed_controller.angular_gain.x",
        "angular_speed_controller.angular_gain.y",
        "angular_speed_controller.angular_gain.z",
    };

  private:
    void readParameters(std::vector<std::string> &params);

    void initialize_references();
    void reset_references();
    void resetErrors();
    void resetCommands();

    void computeActions(
        geometry_msgs::msg::PoseStamped &pose,
        geometry_msgs::msg::TwistStamped &twist,
        as2_msgs::msg::Thrust &thrust);

    void computeHOVER(
        geometry_msgs::msg::PoseStamped &pose,
        geometry_msgs::msg::TwistStamped &twist,
        as2_msgs::msg::Thrust &thrust);

    void computeSpeedControl(Vector3d &f_des);
    void computeTrajectoryControl(Vector3d &f_des);

    void computeYawAngleControl(Vector3d &f_des, Vector3d &acro, float &thrust);
    void computeYawSpeedControl(Vector3d &f_des, Vector3d &acro, float &thrust);

    void getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                   geometry_msgs::msg::TwistStamped &twist_msg,
                   as2_msgs::msg::Thrust &thrust_msg,
                   Vector3d &acro,
                   float &thrust);
  };
};

#endif
