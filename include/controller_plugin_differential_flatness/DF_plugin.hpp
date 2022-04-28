#ifndef __DF_PLUGIN_H__
#define __DF_PLUGIN_H__

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

// #include "as2_msgs/srv/set_controller_control_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <Eigen/Dense>
#include "Eigen/src/Core/Matrix.h"

#include "controller_plugin_base/controller_base.hpp"

#include "DF_controller.hpp"

namespace controller_plugin_differential_flatness
{
  using Vector3d = Eigen::Vector3d;

  struct Control_flags
  {
    bool ref_generated;
    bool hover_position;
    bool state_received;
    bool parameters_read;
  };

  struct UAV_state
  {
    // State composed of s = [pose ,d_pose]'
    Vector3d pos;
    Vector3d rot;
    Vector3d vel;
  };

  class DFPlugin : public controller_plugin_base::ControllerBase
  {
  public:
    DFPlugin(){};
    ~DFPlugin(){};

  public:
    void ownInitialize() override;
    void updateState(const nav_msgs::msg::Odometry &odom) override;
    void updateReference(const geometry_msgs::msg::TwistStamped &ref) override;
    void updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &ref) override;

    void computeOutput(geometry_msgs::msg::PoseStamped &pose,
                       geometry_msgs::msg::TwistStamped &twist,
                       as2_msgs::msg::Thrust &thrust) override;

    bool setMode(const as2_msgs::msg::ControlMode &mode_in,
                 const as2_msgs::msg::ControlMode &mode_out) override;

    void get_default_parameters();
    void update_gains(const std::unordered_map<std::string, double> &params);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  private:
    rclcpp::Time last_time_;

    as2_msgs::msg::ControlMode control_mode_in_;
    as2_msgs::msg::ControlMode control_mode_out_;

    UAV_state state_;
    Control_flags flags_;

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

    Eigen::Matrix3d Rot_matrix = Eigen::Matrix3d::Identity();
    float antiwindup_cte_ = 1.0f;

    Vector3d f_des_ = Vector3d::Zero();
    Vector3d acro_ = Vector3d::Zero();
    float thrust_ = 0.0;

    std::array<std::array<float, 3>, 4> refs_;

    // List of string names for the parameters

    std::unordered_map<std::string, double> parameters_ = {
        {"uav_mass", 1.5},
        {"antiwindup_cte", 1.0},
        {"speed_following.speed_Kp.x", 3.0},
        {"speed_following.speed_Kp.y", 3.0},
        {"speed_following.speed_Kp.z", 4.0},
        {"speed_following.speed_Kd.x", 0.0},
        {"speed_following.speed_Kd.y", 0.0},
        {"speed_following.speed_Kd.z", 0.0},
        {"speed_following.speed_Ki.x", 0.0},
        {"speed_following.speed_Ki.y", 0.0},
        {"speed_following.speed_Ki.z", 0.01},
        {"trajectory_following.position_Kp.x", 6.0},
        {"trajectory_following.position_Kp.y", 6.0},
        {"trajectory_following.position_Kp.z", 6.0},
        {"trajectory_following.position_Kd.x", 0.01},
        {"trajectory_following.position_Kd.y", 0.01},
        {"trajectory_following.position_Kd.z", 0.01},
        {"trajectory_following.position_Ki.x", 3.0},
        {"trajectory_following.position_Ki.y", 3.0},
        {"trajectory_following.position_Ki.z", 3.0},
        {"angular_speed_controller.angular_gain.x", 5.5},
        {"angular_speed_controller.angular_gain.y", 5.5},
        {"angular_speed_controller.angular_gain.z", 5.0},
    };

  private:
    void declareParameters(std::unordered_map<std::string, double> &params);

    void resetState();
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

    Vector3d computeSpeedControl();
    Vector3d computeTrajectoryControl();

    void computeYawAngleControl(Vector3d &acro, float &thrust);
    void computeYawSpeedControl(Vector3d &acro, float &thrust);

    void getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                   geometry_msgs::msg::TwistStamped &twist_msg,
                   as2_msgs::msg::Thrust &thrust_msg);
  };
};

#endif
