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
    Vector3d pos;
    Vector3d vel;
    Eigen::Matrix3d rot;
  };

  struct Control_ref
  {
    Vector3d pos;
    Vector3d vel;
    Vector3d acc;
    Vector3d yaw;
  };

  class DFController
  {
  public:
    DFController();
    ~DFController(){};

  public:
    // void set_uav_state(const UAV_state& uav_state);
    // void set_references(const Control_ref& control_ref);

    Eigen::Vector3d getPositionError();
    Eigen::Vector3d getVelocityError();
    void resetError();

    bool setParameter(const std::string &param, const double &value);
    bool getParameter(const std::string &param, double &value);
    bool isParameter(const std::string &param);
    bool setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list);
    std::vector<std::pair<std::string, double>> getParametersList();

    Vector3d computeVelocityControl(
      const UAV_state &state_,
      const Control_ref &ref_,
      const double &dt
    );

    Vector3d computeTrajectoryControl(
      const UAV_state &state,
      const Control_ref &ref,
      const double &dt
    );

    void computeYawAngleControl(
      // Input
      const UAV_state &state,
      const float &yaw_angle_ref,
      const Vector3d &force_des,
      // Output
      Vector3d &acro, 
      float &thrust
    );

    void computeYawSpeedControl(
      // Input
      const UAV_state &state,
      const float &yaw_speed_ref,
      const Vector3d &force_des,
      const double &dt,
      // Output
      Vector3d &acro, 
      float &thrust
    );

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
    
    Vector3d f_des_ = Vector3d::Zero();
    Vector3d acro_ = Vector3d::Zero();
    float thrust_ = 0.0;

    float mass_ = 1.5f;
    float antiwindup_cte_ = 1.0f;
    double alpha_ = 0.1;
  
  private:
    void updateGains_();
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
  using Control_ref = differential_flatness_controller::Control_ref;

  struct Control_flags
  {
    bool parameters_read;
    bool state_received;
    bool ref_received;
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

    UAV_state uav_state_;
    Control_ref control_ref_;

    Vector3d f_des_ = Vector3d::Zero();
    Vector3d acro_ = Vector3d::Zero();
    float thrust_ = 0.0;

    std::vector<std::string> parameters_to_read_ = {
      "uav_mass",
      "antiwindup_cte",
      "alpha",
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
    void declareParameters();

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
    void resetReferences();
    void resetCommands();
  };
};

#endif
#pragma endregion