/*!*******************************************************************************************
 *  \file       DF_controller_plugin.cpp
 *  \brief      Differential flatness controller plugin for the Aerostack framework.
 *  \authors    Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
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
 ********************************************************************************/

#include "DF_controller_plugin.hpp"

namespace controller_plugin_differential_flatness {

void Plugin::ownInitialize() {
  flags_.parameters_read = false;
  flags_.state_received  = false;
  flags_.ref_received    = false;

  std::shared_ptr<df_controller::DF> controller_handler_ = std::make_shared<df_controller::DF>();

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  parameters_to_read_ = std::vector<std::string>(parameters_list_);

  reset();
  return;
};

bool Plugin::updateParams(const std::vector<std::string> &_params_list) {
  auto result = parametersCallback(node_ptr_->get_parameters(_params_list));
  return result.successful;
};

rcl_interfaces::msg::SetParametersResult Plugin::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "success";

  for (auto &param : parameters) {
    std::string param_name = param.get_name();

    std::string controller    = param_name.substr(0, param_name.find("."));
    std::string param_subname = param_name.substr(param_name.find(".") + 1);
    if (controller == "trajectory_control") {
      updateDFParameter(param_subname, param);
    }

    if (!flags_.parameters_read && find(parameters_to_read_.begin(), parameters_to_read_.end(),
                                        param.get_name()) != parameters_to_read_.end()) {
      // Remove the parameter from the list of parameters to be read
      parameters_to_read_.erase(
          std::remove(parameters_to_read_.begin(), parameters_to_read_.end(), param.get_name()),
          parameters_to_read_.end());

      if (parameters_to_read_.size() == 0) {
        RCLCPP_INFO(node_ptr_->get_logger(), "All parameters read");
        flags_.parameters_read = true;
      }
    }
  }
  return result;
}

void Plugin::updateDFParameter(const std::string &_parameter_name,
                               const rclcpp::Parameter &_param) {
  if (_parameter_name == "reset_integral") {
    controller_handler_->setResetIntegralSaturationFlag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    controller_handler_->setAntiWindup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    controller_handler_->setAlpha(_param.get_value<double>());
  } else if (_parameter_name == "kp.x") {
    controller_handler_->setGainKpX(_param.get_value<double>());
  } else if (_parameter_name == "kp.y") {
    controller_handler_->setGainKpY(_param.get_value<double>());
  } else if (_parameter_name == "kp.z") {
    controller_handler_->setGainKpZ(_param.get_value<double>());
  } else if (_parameter_name == "ki.x") {
    controller_handler_->setGainKiX(_param.get_value<double>());
  } else if (_parameter_name == "ki.y") {
    controller_handler_->setGainKiY(_param.get_value<double>());
  } else if (_parameter_name == "ki.z") {
    controller_handler_->setGainKiZ(_param.get_value<double>());
  } else if (_parameter_name == "kd.x") {
    controller_handler_->setGainKdX(_param.get_value<double>());
  } else if (_parameter_name == "kd.y") {
    controller_handler_->setGainKdY(_param.get_value<double>());
  } else if (_parameter_name == "kd.z") {
    controller_handler_->setGainKdZ(_param.get_value<double>());
  } else if (_parameter_name == "roll_control.kp") {
    controller_handler_->setGainKpRollAngular(_param.get_value<double>());
  } else if (_parameter_name == "pitch_control.kp") {
    controller_handler_->setGainKpPitchAngular(_param.get_value<double>());
  } else if (_parameter_name == "yaw_control.kp") {
    controller_handler_->setGainKpYawAngular(_param.get_value<double>());
  }
  return;
}

void Plugin::reset() {
  resetState();
  resetReferences();
  resetCommands();
  controller_handler_->resetController();
  last_time_ = node_ptr_->now();
}

void Plugin::resetState() {
  uav_state_ = UAV_state();
  return;
}

void Plugin::resetReferences() {
  control_ref_.position_header = uav_state_.position_header;
  control_ref_.position        = uav_state_.position;

  control_ref_.velocity_header = uav_state_.velocity_header;
  control_ref_.velocity        = Eigen::Vector3d::Zero();

  control_ref_.acceleration_header = uav_state_.velocity_header;
  control_ref_.acceleration        = Eigen::Vector3d::Zero();

  control_ref_.yaw =
      Eigen::Vector3d(as2::frame::getYawFromQuaternion(uav_state_.attitude_state), 0, 0);
  return;
}

void Plugin::resetCommands() {
  control_command_.PQR    = Eigen::Vector3d::Zero();
  control_command_.thrust = 0.0;
  return;
}

void Plugin::updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                         const geometry_msgs::msg::TwistStamped &twist_msg) {
  uav_state_.position_header = pose_msg.header;
  uav_state_.position =
      Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

  geometry_msgs::msg::TwistStamped twist_msg_flu = twist_msg;
  geometry_msgs::msg::TwistStamped twist_msg_enu;
  twist_msg_enu       = tf_handler_->convert(twist_msg_flu, enu_frame_id_);
  uav_state_.velocity = Eigen::Vector3d(twist_msg_enu.twist.linear.x, twist_msg_enu.twist.linear.y,
                                        twist_msg_enu.twist.linear.z);

  uav_state_.attitude_state =
      tf2::Quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                      pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

  flags_.state_received = true;
  return;
};

void Plugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg) {
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  control_ref_.position =
      Eigen::Vector3d(traj_msg.positions[0], traj_msg.positions[1], traj_msg.positions[2]);

  control_ref_.velocity =
      Eigen::Vector3d(traj_msg.velocities[0], traj_msg.velocities[1], traj_msg.velocities[2]);

  control_ref_.acceleration = Eigen::Vector3d(traj_msg.accelerations[0], traj_msg.accelerations[1],
                                              traj_msg.accelerations[2]);

  control_ref_.yaw =
      Eigen::Vector3d(traj_msg.positions[3], traj_msg.velocities[3], traj_msg.accelerations[3]);

  flags_.ref_received = true;
  return;
};

bool Plugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                     const as2_msgs::msg::ControlMode &out_mode) {
  if (in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    control_mode_in_.control_mode    = in_mode.control_mode;
    control_mode_in_.yaw_mode        = as2_msgs::msg::ControlMode::YAW_ANGLE;
    control_mode_in_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  } else {
    flags_.ref_received   = false;
    flags_.state_received = false;
    control_mode_in_      = in_mode;
  }

  control_mode_out_ = out_mode;
  reset();

  return true;
};

void Plugin::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                           geometry_msgs::msg::TwistStamped &twist,
                           as2_msgs::msg::Thrust &thrust) {
  if (!flags_.state_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
    return;
  }

  if (!flags_.parameters_read) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Parameters not read yet");
    return;
  }

  if (!flags_.ref_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "State changed, but ref not recived yet");
    return;
  } else {
    computeActions(pose, twist, thrust);
  }

  static rclcpp::Time last_time_ = node_ptr_->now();
  return;
};

void Plugin::computeActions(geometry_msgs::msg::PoseStamped &pose,
                            geometry_msgs::msg::TwistStamped &twist,
                            as2_msgs::msg::Thrust &thrust) {
  rclcpp::Time current_time = node_ptr_->now();
  double dt                 = (current_time - last_time_).nanoseconds() / 1.0e9;
  last_time_                = current_time;

  if (dt == 0) {
    // Send last command reference
    RCLCPP_WARN_ONCE(node_ptr_->get_logger(), "Loop delta time is zero");
  } else {
    resetCommands();

    switch (control_mode_in_.yaw_mode) {
      case as2_msgs::msg::ControlMode::YAW_ANGLE: {
        break;
      }
      case as2_msgs::msg::ControlMode::YAW_SPEED: {
        tf2::Matrix3x3 m(uav_state_.attitude_state);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        control_ref_.yaw.x() = yaw + control_ref_.yaw.y() * dt;
        break;
      }
      default:
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown yaw mode");
        return;
        break;
    }

    switch (control_mode_in_.control_mode) {
      case as2_msgs::msg::ControlMode::HOVER: {
        // TODO: Implement
        // control_command_.velocity = pid_3D_position_handler_->computeControl(
        //     dt, uav_state_.position, control_ref_.position);
        break;
      }
      case as2_msgs::msg::ControlMode::POSITION: {
        // TODO: Implement
        // control_command_.velocity = pid_3D_position_handler_->computeControl(
        //     dt, uav_state_.position, control_ref_.position);
        break;
      }
      case as2_msgs::msg::ControlMode::SPEED: {
        // TODO: Implement
        // control_command_.velocity = pid_3D_velocity_handler_->computeControl(
        //     dt, uav_state_.velocity, control_ref_.velocity);
        break;
      }
      case as2_msgs::msg::ControlMode::TRAJECTORY: {
        // TODO: Change twist to odom frame
        control_command_ = controller_handler_->computeTrajectoryControl(
            dt, uav_state_.position, uav_state_.velocity, uav_state_.attitude_state,
            control_ref_.position, control_ref_.velocity, control_ref_.acceleration,
            control_ref_.yaw.x());
        break;
      }
      default:
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown control mode");
        return;
        break;
    }

    switch (control_mode_in_.reference_frame) {
      case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
        getOutput(twist, thrust);
        break;

      default:
        RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Unknown reference frame");
        return;
        break;
    }
  }
  return;
}

void Plugin::getOutput(geometry_msgs::msg::TwistStamped &twist_msg,
                       as2_msgs::msg::Thrust &thrust_msg) {
  twist_msg.header.stamp    = node_ptr_->now();
  twist_msg.header.frame_id = flu_frame_id_;
  twist_msg.twist.angular.x = control_command_.PQR.x();
  twist_msg.twist.angular.y = control_command_.PQR.y();
  twist_msg.twist.angular.z = control_command_.PQR.z();

  thrust_msg.header.stamp    = node_ptr_->now();
  thrust_msg.header.frame_id = flu_frame_id_;
  thrust_msg.thrust          = control_command_.thrust;
  return;
};

}  // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_differential_flatness::Plugin,
                       controller_plugin_base::ControllerBase)
