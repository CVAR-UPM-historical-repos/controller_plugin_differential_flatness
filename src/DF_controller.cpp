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

PD_controller::PD_controller() : as2::Node("differential_flatness_controller")
{
  //TODO: read drone config files to read mass parameters
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->generate_global_name("self_localization/odom"), 1,
    std::bind(&PD_controller::CallbackOdomTopic, this, std::placeholders::_1));
  sub_traj_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    this->generate_global_name("motion_reference/trajectory"), 1,
    std::bind(&PD_controller::CallbackTrajTopic, this, std::placeholders::_1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    this->generate_global_name("platform/imu"), 1,
    std::bind(&PD_controller::CallbackImuTopic, this, std::placeholders::_1));
}

void PD_controller::setup()
{
  RCLCPP_INFO(this->get_logger(), "PD controller non-linearized");
  RCLCPP_INFO(this->get_logger(), "uav_mass = %f", mass);

  Kp_lin_ << 5.0, 5.0, 6.0;
  Kd_lin_ << 3.0, 3.0, 3.0;
  Ki_lin_ << 0.01, 0.01, 0.01;
  accum_error_ << 0, 0, 0;

  Kp_ang_ << 5.5, 5.5, 5.0;

  flags_.traj_generated = false;
  flags_.hover_position = false;
  flags_.state_received = false;

  //set all refs to zefs
  for (auto dof : refs_) {
    for (auto elem : dof) elem = 0.0f;
  }
}

void PD_controller::computeActions()
{
  static Eigen::Matrix3d Kp_lin_mat = Kp_lin_.asDiagonal();
  static Eigen::Matrix3d Kd_lin_mat = Kd_lin_.asDiagonal();
  static Eigen::Matrix3d Kp_ang_mat = Kp_ang_.asDiagonal();
  static Eigen::Matrix3d Ki_lin_mat = Ki_lin_.asDiagonal();
  const static Eigen::Vector3d gravitational_force(0, 0, mass * g);

  Vector3d r(state_.pos[0], state_.pos[1], state_.pos[2]);
  Vector3d rdot(state_.vel[0], state_.vel[1], state_.vel[2]);
  Vector3d r_t(refs_[0][0], refs_[1][0], refs_[2][0]);
  Vector3d rdot_t(refs_[0][1], refs_[1][1], refs_[2][1]);
  Vector3d rddot_t(refs_[0][2], refs_[1][2], refs_[2][2]);

  Vector3d e_p = r - r_t;
  Vector3d e_v = rdot - rdot_t;

  accum_error_ += e_p;

  float antiwindup_cte = 1.0f;
  for (short j = 0; j < 3; j++) {
    float antiwindup_value = antiwindup_cte / Ki_lin_[j];
    accum_error_[j] = (accum_error_[j] > antiwindup_value) ? antiwindup_value : accum_error_[j];
    accum_error_[j] = (accum_error_[j] < -antiwindup_value) ? -antiwindup_value : accum_error_[j];
  }

  Vector3d F_des = -Kp_lin_mat * e_p - Ki_lin_mat * accum_error_ - Kd_lin_mat * e_v +
                   gravitational_force + mass * rddot_t;

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

  u2[0] = outputs(0);  // ROLL
  u2[1] = outputs(1);  // PITCH
  u2[2] = outputs(2);  // YAW
};

void PD_controller::publishActions()
{
  static as2::controlCommandsHandlers::AcroControl acro_controller(this);
  // FIXME: check sings
  acro_controller.sendAngleRatesWithThrust(u2[0], -u2[1], -u2[2], u1);
  // RCLCPP_INFO(this->get_logger(), "Publishing actions");
};

void PD_controller::run()
{
  if (flags_.traj_generated) {
    this->computeActions();
    this->publishActions();
  }
};

/* --------------------------- CALLBACKS ---------------------------*/

void PD_controller::CallbackTrajTopic(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  auto & traj_msg = *(msg.get());
  flags_.traj_generated = true;
  
  for (int i = 0; i < 4; i++) {
    refs_[i][0] = traj_msg.positions[i];
    refs_[i][1] = traj_msg.velocities[i];
    refs_[i][2] = traj_msg.accelerations[i];
  }
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

void PD_controller::CallbackImuTopic(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto & imu_msg = *(msg.get());

  // FIXME: IMU IS FRD NOT ENU
  //FIXME SIGNS ARE CHANGED
  state_.omega[0] = imu_msg.angular_velocity.x;
  state_.omega[1] = -imu_msg.angular_velocity.y;
  state_.omega[2] = -imu_msg.angular_velocity.z;
}
