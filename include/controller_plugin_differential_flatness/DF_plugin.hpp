#pragma region DF_controller

#ifndef __DF_CONTROLLER_H__
#define __DF_CONTROLLER_H__

#include <iostream>
#include <Eigen/Dense>
#include "Eigen/src/Core/Matrix.h"

namespace differential_flatness_controller
{
  using Vector3d = Eigen::Vector3d;

  struct UAV_state
  {
    // State composed of s = [pose ,d_pose]'
    Vector3d pos;
    Vector3d rot;
    Vector3d vel;
  };

  struct Control_ref
  {
    // State composed of s = [pose ,d_pose]'
    Vector3d pos;
    Vector3d vel;
    Vector3d acc;
    Vector3d yaw;
  };

  class DFController
  {
  public:
    DFController(const UAV_state& uav_state);
    ~DFController(){};

  public:
    uint32_t last_time;
    uint32_t current_time;

  public:
    void set_UAV_State(const UAV_state& uav_state);
    void set_references(const Control_ref& control_ref);

    Eigen::Vector3d get_position_error();
    Eigen::Vector3d get_velocity_error();
    void reset_error();

    bool set_parameter(const std::string& param, const double& value);
    bool get_parameter(const std::string& param, double& value);
    bool set_Parameter_list(const std::vector<std::pair<std::string, double>> parameter_list);
    std::vector<std::pair<std::string,double>> get_parameters_list();

    Vector3d compute_velocity_control(const double& dt);
    Vector3d compute_trajectory_control(const double& dt);

    void computeYawAngleControl(Vector3d &acro, float &thrust);
    void computeYawSpeedControl(Vector3d &acro, float &thrust);

  private:
    UAV_state state_;
    Control_ref ref_;

    Eigen::Vector3d position_accum_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_accum_error_ = Eigen::Vector3d::Zero();

    const float g = 9.81;
    const Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, g);

    std::unordered_map<std::string, double> parameters_ = {
      {"uav_mass", 1.5},
      {"antiwindup_cte", 1.0},
      {"alpha", 0.1},
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

    Eigen::Matrix3d traj_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d traj_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d traj_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d velocity_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d velocity_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d velocity_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Kp_ang_mat_ = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Rot_matrix_ = Eigen::Matrix3d::Identity();
    
    Vector3d f_des_ = Vector3d::Zero();
    Vector3d acro_ = Vector3d::Zero();
    float thrust_ = 0.0;

    float mass_ = 1.5f;
    float antiwindup_cte_ = 1.0f;
    double alpha_ = 0.1;
  
  private:
    void update_gains_();
  };
};

#endif


#pragma endregion


#pragma region DF_PLUGIN

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

#include "controller_plugin_base/controller_base.hpp"

// #include "DF_controller.hpp"

namespace controller_plugin_differential_flatness
{
  using Vector3d = Eigen::Vector3d;
  using DFController = differential_flatness_controller::DFController;
  using UAV_state = differential_flatness_controller::UAV_state;

  struct Control_flags
  {
    bool ref_generated;
    bool hover_position;
    bool state_received;
    bool parameters_read;
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

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  private:
    rclcpp::Time last_time_;

    as2_msgs::msg::ControlMode control_mode_in_;
    as2_msgs::msg::ControlMode control_mode_out_;

    Control_flags flags_;

    std::shared_ptr<DFController> controller_handler_;

  private:
    void set_default_parameters();
    void declareParameters(std::unordered_map<std::string, double> &params);

    void computeActions(
        geometry_msgs::msg::PoseStamped &pose,
        geometry_msgs::msg::TwistStamped &twist,
        as2_msgs::msg::Thrust &thrust);

    void computeHOVER(
        geometry_msgs::msg::PoseStamped &pose,
        geometry_msgs::msg::TwistStamped &twist,
        as2_msgs::msg::Thrust &thrust);
    
    void getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                   geometry_msgs::msg::TwistStamped &twist_msg,
                   as2_msgs::msg::Thrust &thrust_msg);

    void resetState();
    void initialize_references();
    void reset_references();
    void reset_commands();
  };
};

#endif
#pragma endregion