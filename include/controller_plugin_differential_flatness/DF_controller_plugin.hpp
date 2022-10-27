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

#include <pid_controller/PID_3D.hpp>

namespace controller_plugin_differential_flatness {

struct UAV_state {
  std_msgs::msg::Header position_header = std_msgs::msg::Header();
  Eigen::Vector3d position              = Eigen::Vector3d::Zero();
  std_msgs::msg::Header velocity_header = std_msgs::msg::Header();
  Eigen::Vector3d velocity              = Eigen::Vector3d::Zero();
  tf2::Quaternion attitude_state        = tf2::Quaternion::getIdentity();
};

struct UAV_reference {
  Eigen::Vector3d position     = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d yaw          = Eigen::Vector3d::Zero();
};

struct Acro_command {
  Eigen::Vector3d PQR;
  double thrust;
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

  // void updateReference(const geometry_msgs::msg::PoseStamped &ref) override;
  // void updateReference(const geometry_msgs::msg::TwistStamped &ref) override;reset
  void updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &ref) override;

  bool setMode(const as2_msgs::msg::ControlMode &mode_in,
               const as2_msgs::msg::ControlMode &mode_out) override;

  bool computeOutput(const double &dt,
                     geometry_msgs::msg::PoseStamped &pose,
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

  std::vector<std::string> parameters_list_ = {
      "mass",
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

  Eigen::Matrix3d Kp_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Kd_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Ki_{Eigen::Matrix3d::Zero()};

  UAV_state uav_state_;
  UAV_reference control_ref_;
  Acro_command control_command_;

  bool proportional_limitation_ = false;
  std::string enu_frame_id_     = "odom";
  std::string flu_frame_id_     = "base_link";

private:
  void checkParamList(const std::string &param,
                      std::vector<std::string> &_params_list,
                      bool &_all_params_read);

  void updateDFParameter(const std::string &_parameter_name, const rclcpp::Parameter &_param);

  void resetState();
  void resetReferences();
  void resetCommands();

  void computeActions(geometry_msgs::msg::PoseStamped &pose,
                      geometry_msgs::msg::TwistStamped &twist,
                      as2_msgs::msg::Thrust &thrust);

  bool getOutput(geometry_msgs::msg::TwistStamped &twist_msg, as2_msgs::msg::Thrust &thrust_msg);

private:
  // std::shared_ptr<pid_controller::PIDController3D> pid_handler_;

  double mass_;
  Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, -9.81);
  Eigen::Matrix3d Kp_ang_mat_          = Eigen::Matrix3d::Zero();

private:
  Eigen::Vector3d getForce(const double &_dt,
                           const Eigen::Vector3d &_pos_state,
                           const Eigen::Vector3d &_vel_state,
                           const Eigen::Vector3d &_pos_reference,
                           const Eigen::Vector3d &_vel_reference,
                           const Eigen::Vector3d &_acc_reference);

  Acro_command computeTrajectoryControl(const double &_dt,
                                        const Eigen::Vector3d &_pos_state,
                                        const Eigen::Vector3d &_vel_state,
                                        const tf2::Quaternion &_attitude_state,
                                        const Eigen::Vector3d &_pos_reference,
                                        const Eigen::Vector3d &_vel_reference,
                                        const Eigen::Vector3d &_acc_reference,
                                        const double &_yaw_angle_reference);
};
};  // namespace controller_plugin_differential_flatness

#endif
