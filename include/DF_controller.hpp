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

//  ros


// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <memory>

// Eigen
#include <Eigen/Dense>

// Std libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <array>

// TODO:dynamic reconfigure
// #include <differential_flatness_controller/ControllerConfig.h>
// #include <dynamic_reconfigure/server.h>

#include "as2_core/node.hpp"
#include "as2_control_command_handlers/acro_control.hpp"

// definitions 

#define DEBUG 0
#define LINEARIZATION 0

// FIXME: read this from the parameter server


#define DRONE_MASS 1.5
// #define DYNAMIC_TUNING
#define SATURATE_YAW_ERROR 1 

using Vector3d = Eigen::Vector3d;


struct Control_flags{
    bool traj_generated;
    bool hover_position;
    bool state_received;
};

struct UAV_state{
    // State composed of s = [pose ,d_pose]'
    float pos[3];
    float rot[3];
    float vel[3];
    float omega[3];
};


class PD_controller : public as2::Node{
private:

    std::string n_space_;
    std::string self_localization_pose_topic_;
    std::string self_localization_speed_topic_;
    std::string sensor_measurement_imu_topic_;
    std::string motion_reference_traj_topic_;
    
    std::string actuator_command_thrust_topic_;
    std::string actuator_command_speed_topic_;

    float mass = 1.0f;    

    
    
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr sub_traj_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    // rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr pub_thrust_;
    // rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_speeds_;
    
    UAV_state state_;
    Control_flags flags_;
    
    // controller stuff
    const float g = 9.81;
    const float angle_limit = 1.0; // pi/4 < value < pi/2 

    
    Eigen::Vector3d Kp_lin_;
    Eigen::Vector3d Kd_lin_;
    Eigen::Vector3d Kp_ang_;
    Eigen::Vector3d Ki_lin_;
    Eigen::Vector3d accum_error_;

    Eigen::Matrix3d Rot_matrix;
    
    float u1 = 0.0;
    float u2[3] = {0.0,0.0,0.0};

    std::array<std::array<float,3>,4> refs_;

public:
    PD_controller();
    ~PD_controller(){};

    void updateErrors();
    void computeActions();
    void publishActions();

    //TODO: CHECK DYNAMIC_TUNING ROS2
    #ifdef DYNAMIC_TUNING
    void parametersCallback(pd_controller::ControllerConfig &config, uint32_t level);
    #endif

    void setup();
    void run();

private:

    void followTrajectory();
    void hover();
    
    void CallbackTrajTopic(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr traj_msg);
    void CallbackOdomTopic(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void CallbackImuTopic(const sensor_msgs::msg::Imu::SharedPtr imu_msg);


};

#endif