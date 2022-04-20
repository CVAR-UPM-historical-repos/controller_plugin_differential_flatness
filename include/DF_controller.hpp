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
#include <Eigen/Dense>

#include "Eigen/src/Core/Matrix.h"
#include "as2_control_command_handlers/acro_control.hpp"
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

#include "as2_msgs/msg/controller_control_mode.hpp"
#include "as2_msgs/srv/set_controller_control_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"

// FIXME: read this from the parameter server

#define SATURATE_YAW_ERROR 1

#define SPEED_REFERENCE 0

using Vector3d = Eigen::Vector3d;

struct Control_flags {
  bool ref_generated;
  bool hover_position;
  bool state_received;
};

struct UAV_state {
  // State composed of s = [pose ,d_pose]'
  Vector3d pos;
  Vector3d rot;
  Vector3d vel;
};

enum ControlMode {
  UNSET = 0,
  HOVER = 1,
  TRAJECTORY = 2,
  SPEED = 3,
};

class PD_controller : public as2::Node {
  private:
    float mass = 1.5f;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr sub_traj_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    UAV_state state_;
    Control_flags flags_;

    // controller stuff
    const float g = 9.81;

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

    std::array<std::array<float, 3>, 4> refs_;

    uint8_t control_mode_ = ControlMode::UNSET;
    std::unordered_map<std::string,double> parameters_;
    rclcpp::Service<as2_msgs::srv::SetControllerControlMode>::SharedPtr set_control_mode_srv_;

  public:
    PD_controller();
    ~PD_controller(){};

    void setup();
    void run();

    void update_gains(const std::unordered_map<std::string,double>& params);
    

  private:

    Vector3d computeForceDesiredByTraj();
    Vector3d computeForceDesiredBySpeed();

    void reset_references();

    void computeActions(uint8_t control_mode);
    void publishActions();

    bool setControlMode(const as2_msgs::msg::ControllerControlMode & msg);

    /* --------------------------- CALLBACKS ---------------------------*/
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    void CallbackTrajTopic(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr traj_msg);
    void CallbackTwistTopic(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);
    void CallbackOdomTopic(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    void setControlModeSrvCall(
      const std::shared_ptr<as2_msgs::srv::SetControllerControlMode::Request> request,
      std::shared_ptr<as2_msgs::srv::SetControllerControlMode::Response> response);
};

#endif
