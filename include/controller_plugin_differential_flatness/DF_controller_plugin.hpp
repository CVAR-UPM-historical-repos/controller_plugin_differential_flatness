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

namespace controller_plugin_differential_flatness {

struct UAV_state {
  Eigen::Vector3d position       = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity       = Eigen::Vector3d::Zero();
  tf2::Quaternion attitude_state = tf2::Quaternion::getIdentity();
};

struct UAV_reference {
  Eigen::Vector3d position     = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d yaw          = Eigen::Vector3d::Zero();
};

struct Acro_command {
  Eigen::Vector3d PQR = Eigen::Vector3d::Zero();
  double thrust       = 0.0;
};

struct Control_flags {
  bool parameters_read = false;
  bool state_received  = false;
  bool ref_received    = false;
};

class Plugin : public controller_plugin_base::ControllerBase {
  UAV_state uav_state_;
  UAV_reference control_ref_;
  Acro_command control_command_;
  Control_flags flags_;

  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  Eigen::Matrix3d Kp_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Kd_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Ki_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Kp_ang_mat_{Eigen::Matrix3d::Zero()};

  Eigen::Vector3d accum_pos_error_{Eigen::Vector3d::Zero()};

  double mass_;
  double antiwindup_cte_ = 0.0;

  const std::string odom_frame_id_      = "odom";
  const std::string base_link_frame_id_ = "base_link";

  const Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, -9.81);

  const std::vector<std::string> parameters_list_ = {
      "mass",
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
  std::vector<std::string> parameters_to_read_{parameters_list_};  // copy mutable

public:
  Plugin(){};
  ~Plugin(){};

  /** Virtual functions from ControllerBase */
  void ownInitialize() override;
  void updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                   const geometry_msgs::msg::TwistStamped &twist_msg) override;

  void updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &ref) override;

  bool setMode(const as2_msgs::msg::ControlMode &mode_in,
               const as2_msgs::msg::ControlMode &mode_out) override;

  bool computeOutput(double dt,
                     geometry_msgs::msg::PoseStamped &pose,
                     geometry_msgs::msg::TwistStamped &twist,
                     as2_msgs::msg::Thrust &thrust) override;

  bool updateParams(const std::vector<std::string> &_params_list) override;
  void reset() override;

  // IMPORTANT: this is the frame_id of the desired pose and twist
  std::string getDesiredPoseFrameId() override { return odom_frame_id_; }
  std::string getDesiredTwistFrameId() override { return odom_frame_id_; }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

private:
  /** Controller especific functions */
  bool checkParamList(const std::string &param, std::vector<std::string> &_params_list);

  void updateDFParameter(std::string _parameter_name, const rclcpp::Parameter &_param);

  void resetState();
  void resetReferences();
  void resetCommands();

  void computeActions(geometry_msgs::msg::PoseStamped &pose,
                      geometry_msgs::msg::TwistStamped &twist,
                      as2_msgs::msg::Thrust &thrust);

  bool getOutput(geometry_msgs::msg::TwistStamped &twist_msg, as2_msgs::msg::Thrust &thrust_msg);

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
