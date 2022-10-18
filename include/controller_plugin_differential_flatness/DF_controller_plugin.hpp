#ifndef __DF_PLUGIN_H__
#define __DF_PLUGIN_H__

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/msg/thrust.hpp>
#include <controller_plugin_base/controller_base.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <differential_flatness_based_controller/differential_flatness_based_controller.hpp>

namespace controller_plugin_differential_flatness {

struct UAV_state {
  std_msgs::msg::Header position_header = std_msgs::msg::Header();
  Eigen::Vector3d position              = Eigen::Vector3d::Zero();
  std_msgs::msg::Header velocity_header = std_msgs::msg::Header();
  Eigen::Vector3d velocity              = Eigen::Vector3d::Zero();
  tf2::Quaternion attitude_state        = tf2::Quaternion::getIdentity();
};

struct UAV_reference {
  std_msgs::msg::Header position_header     = std_msgs::msg::Header();
  Eigen::Vector3d position                  = Eigen::Vector3d::Zero();
  std_msgs::msg::Header velocity_header     = std_msgs::msg::Header();
  Eigen::Vector3d velocity                  = Eigen::Vector3d::Zero();
  std_msgs::msg::Header acceleration_header = std_msgs::msg::Header();
  Eigen::Vector3d acceleration              = Eigen::Vector3d::Zero();
  Eigen::Vector3d yaw                       = Eigen::Vector3d::Zero();
};

struct Control_flags {
  bool parameters_read;
  bool state_received;
  bool ref_received;
};

class Plugin : public controller_plugin_base::ControllerBase {
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

  bool setMode(const as2_msgs::msg::ControlMode &mode_in,
               const as2_msgs::msg::ControlMode &mode_out) override;

  void computeOutput(geometry_msgs::msg::PoseStamped &pose,
                     geometry_msgs::msg::TwistStamped &twist,
                     as2_msgs::msg::Thrust &thrust) override;

  bool updateParams(const std::vector<std::string> &_params_list) override;
  void reset() override;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

private:
  rclcpp::Time last_time_;

  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  Control_flags flags_;

  std::shared_ptr<df_controller::DF> controller_handler_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  std::vector<std::string> parameters_list_ = {
      "trajectory_control.reset_integral",
      "trajectory_control.antiwindup_cte",
      "trajectory_control.alpha",
      "trajectory_control.kp.x",
      "trajectory_control.kp.y",
      "trajectory_control.kp.z",
      "trajectory_control.ki.x",
      "trajectory_control.ki.y",
      "trajectory_control.ki.z",
      "trajectory_control.kd.x",
      "trajectory_control.kd.y",
      "trajectory_control.kd.z",
      "trajectory_control.roll_control.kp",
      "trajectory_control.pitch_control.kp",
      "trajectory_control.yaw_control.kp",
  };

  std::vector<std::string> parameters_to_read_;

  UAV_state uav_state_;
  UAV_reference control_ref_;
  df_controller::Acro_command control_command_;

  bool proportional_limitation_ = false;
  std::string enu_frame_id_     = "odom";
  std::string flu_frame_id_     = "base_link";

private:
  void updateDFParameter(const std::string &_parameter_name, const rclcpp::Parameter &_param);

  void resetState();
  void resetReferences();
  void resetCommands();

  void computeActions(geometry_msgs::msg::PoseStamped &pose,
                      geometry_msgs::msg::TwistStamped &twist,
                      as2_msgs::msg::Thrust &thrust);

  void getOutput(geometry_msgs::msg::TwistStamped &twist_msg, as2_msgs::msg::Thrust &thrust_msg);
};
};  // namespace controller_plugin_differential_flatness

#endif
