#ifndef __DF_PLUGIN_H__
#define __DF_PLUGIN_H__

// Std libraries
#include <array>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <vector>

// #include "as2_control_command_handlers/acro_control.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "controller_plugin_base/controller_base.hpp"

#include "DF_controller.hpp"

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

  class Plugin : public controller_plugin_base::ControllerBase
  {
  public:
    Plugin(){};
    ~Plugin(){};

  public:
    void ownInitialize() override;
    void updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                     const geometry_msgs::msg::TwistStamped &twist_msg) override;

    void updateReference(const geometry_msgs::msg::PoseStamped &ref) override;
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
    Control_ref hover_ref_;
    bool in_hover_ = false;
    Vector3d speed_limits_;

    Vector3d f_des_ = Vector3d::Zero();
    Vector3d acro_ = Vector3d::Zero();
    float thrust_ = 0.0;

    bool proportional_limitation_ = false;

    std::vector<std::string> parameters_list_ = {
        "proportional_limitation"
    };

    std::vector<std::string> parameters_to_read_;
    
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