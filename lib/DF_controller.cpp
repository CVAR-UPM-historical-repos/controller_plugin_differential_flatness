#include "DF_controller.hpp"

namespace differential_flatness_controller
{
  DFController::DFController()
  {
    resetError();
    // updateGains_();
  };

  bool DFController::setParameter(const std::string &param, const double &value)
  {
    if (parameters_.count(param) == 1)
    {
      parameters_[param] = value;
      updateGains_();
      return true;
    }
    return false;
  };

  bool DFController::getParameter(const std::string &param, double &value)
  {
    if (parameters_.count(param) == 1)
    {
      value = parameters_[param];
      return true;
    }
    return false;
  };

  bool DFController::isParameter(const std::string &param)
  {
    return parameters_.count(param) == 1;
  };

  bool DFController::setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list)
  {
    for (auto &param : parameter_list)
    {
      if (parameters_.count(param.first) == 1)
      {
        parameters_[param.first] = param.second;
      }
      else
      {
        return false;
      }
    }
    updateGains_();
    return true;
  };

  std::vector<std::pair<std::string, double>> DFController::getParametersList()
  {
    std::vector<std::pair<std::string, double>> list;
    for (auto &param : parameters_)
    {
      list.push_back({param.first, param.second});
    }
    return list;
  };

  Eigen::Vector3d DFController::getPositionError()
  {
    return position_accum_error_;
  };

  Eigen::Vector3d DFController::getVelocityError()
  {
    return position_accum_error_;
  };

  Eigen::Vector3d DFController::getTrajPositionError()
  {
    return traj_position_accum_error_;
  };

  void DFController::resetError()
  {
    position_accum_error_.setZero();
    velocity_accum_error_.setZero();
    traj_position_accum_error_.setZero();
    return;
  };

  Vector3d DFController::computePositionControl(
      const UAV_state &state,
      const Control_ref &ref,
      const double &dt,
      const Vector3d &speed_limit)
  {
    // Compute the proportional contribution (position error)
    Vector3d position_error = ref.pos - state.pos;
    Vector3d p_position_error_contribution = position_Kp_lin_mat_ * position_error;

    // Store the error for the next iteration
    static Vector3d last_position_error_ = position_error;
    static Vector3d filtered_d_position_error_ = position_error;

    // Compute the derivative contribution of the error filtered with a first order filter (position derivate)
    Vector3d position_error_incr = (position_error - last_position_error_);
    filtered_d_position_error_ = alpha_ * position_error_incr + (1.0 - alpha_) * filtered_d_position_error_;

    // Compute the derivate contribution (velocity error)
    Vector3d d_position_error_contribution = position_Kd_lin_mat_ * filtered_d_position_error_ / dt;

    // Update de acumulated error
    position_accum_error_ += position_error * dt;

    // Compute anti-windup. Limit integral contribution
    for (short j = 0; j < 3; j++)
    {
      float antiwindup_value = antiwindup_cte_ / position_Ki_lin_mat_.diagonal()[j];
      position_accum_error_[j] = (position_accum_error_[j] > antiwindup_value) ? antiwindup_value : position_accum_error_[j];
      position_accum_error_[j] = (position_accum_error_[j] < -antiwindup_value) ? -antiwindup_value : position_accum_error_[j];
    }

    // Compute de integral contribution (position integrate)
    Vector3d i_position_error_contribution = position_Ki_lin_mat_ * position_accum_error_;

    // Compute desired acceleration
    Vector3d desired_speed = p_position_error_contribution + d_position_error_contribution + i_position_error_contribution;

    Control_ref desired_ref = ref;
    // Limit speed for each axis
    for (short j = 0; j < 3; j++)
    {
      if (speed_limit[j] == 0.0f)
      {
        continue;
      }
    
      desired_speed[j] = (desired_speed[j] < -speed_limit[j]) ? -speed_limit[j] : desired_speed[j];
      desired_speed[j] = (desired_speed[j] >  speed_limit[j]) ?  speed_limit[j] : desired_speed[j];
    }
    desired_ref.vel = desired_speed;

    return computeVelocityControl(state_, desired_ref, dt);
  };

  Vector3d DFController::computeVelocityControl(
      const UAV_state &state,
      const Control_ref &ref,
      const double &dt)
  {
    // Compute the proportional contribution (velocity)
    Vector3d velocity_error = ref.vel - state.vel;
    Vector3d p_velocity_error_contribution = velocity_Kp_lin_mat_ * velocity_error;

    // Store the error for the next iteration
    static Vector3d last_velocity_error_ = velocity_error;
    static Vector3d filtered_d_velocity_error_ = velocity_error;

    // Compute the derivative contribution of the error filtered with a first order filter (velocity derivate)
    Vector3d velocity_error_incr = (velocity_error - last_velocity_error_);
    filtered_d_velocity_error_ = alpha_ * velocity_error_incr + (1.0 - alpha_) * filtered_d_velocity_error_;
    Vector3d d_velocity_error_contribution = velocity_Kd_lin_mat_ * filtered_d_velocity_error_ / dt;

    // Update the last velocity error
    last_velocity_error_ = velocity_error;

    // Update de acumulated error
    velocity_accum_error_ += velocity_error * dt;

    // Compute anti-windup. Limit integral contribution
    for (short j = 0; j < 3; j++)
    {
      float antiwindup_value = antiwindup_cte_ / velocity_Ki_lin_mat_.diagonal()[j];
      velocity_accum_error_[j] = (velocity_accum_error_[j] > antiwindup_value) ? antiwindup_value : velocity_accum_error_[j];
      velocity_accum_error_[j] = (velocity_accum_error_[j] < -antiwindup_value) ? -antiwindup_value : velocity_accum_error_[j];
    }

    // Compute de integral contribution (velocity integrate)
    Vector3d i_velocity_error_contribution = velocity_Ki_lin_mat_ * velocity_accum_error_;

    // Compute desired acceleration
    Vector3d desired_acceleration = p_velocity_error_contribution + d_velocity_error_contribution + i_velocity_error_contribution;

    // Compute gravity compensation
    Vector3d force_gravity = mass_ * gravitational_accel_;

    // Return desired force with the gravity compensation
    Vector3d desired_force = mass_ * desired_acceleration + force_gravity;

    return desired_force;
  };

  Vector3d DFController::computeTrajectoryControl(
      const UAV_state &state,
      const Control_ref &ref,
      const double &dt)
  {
    // Compute the proportional contribution (position)
    Vector3d position_error = ref.pos - state.pos;
    Vector3d p_error_contribution = traj_Kp_lin_mat_ * position_error;

    // Compute de derivative error contribution (velocity)
    Vector3d velocity_error = ref.vel - state.vel;
    Vector3d d_error_contribution = traj_Kd_lin_mat_ * velocity_error;

    // Update de acumulated error
    traj_position_accum_error_ += position_error;

    // Compute anti-windup. Limit integral contribution
    for (short j = 0; j < 3; j++)
    {
      float antiwindup_value = antiwindup_cte_ / traj_Ki_lin_mat_.diagonal()[j];
      traj_position_accum_error_[j] = (traj_position_accum_error_[j] > antiwindup_value) ? antiwindup_value : traj_position_accum_error_[j];
      traj_position_accum_error_[j] = (traj_position_accum_error_[j] < -antiwindup_value) ? -antiwindup_value : traj_position_accum_error_[j];
    }

    // Compute de integral contribution (position integrate)
    Vector3d i_error_contribution = traj_Ki_lin_mat_ * traj_position_accum_error_;

    // Compute the error force contribution
    Vector3d force_error = p_error_contribution + d_error_contribution + i_error_contribution;

    // Compute acceleration reference contribution
    Vector3d force_acceleration = mass_ * ref.acc;

    // Compute gravity compensation
    Vector3d force_gravity = mass_ * gravitational_accel_;

    // Return desired force with the gravity compensation
    Vector3d desired_force = force_error + force_acceleration + force_gravity;
    return desired_force;
  };

  void DFController::computeYawAngleControl(
      // Input
      const UAV_state &state,
      const float &yaw_angle_ref,
      const Vector3d &force_des,
      // Output
      Vector3d &acro,
      float &thrust)
  {
    tf2::Matrix3x3 rot_matrix_tf2(state.rot);

    Eigen::Matrix3d rot_matrix;
    rot_matrix << rot_matrix_tf2[0][0], rot_matrix_tf2[0][1], rot_matrix_tf2[0][2],
        rot_matrix_tf2[1][0], rot_matrix_tf2[1][1], rot_matrix_tf2[1][2],
        rot_matrix_tf2[2][0], rot_matrix_tf2[2][1], rot_matrix_tf2[2][2];

    Vector3d xc_des(cos(yaw_angle_ref), sin(yaw_angle_ref), 0);

    Vector3d zb_des = force_des.normalized();
    Vector3d yb_des = zb_des.cross(xc_des).normalized();
    Vector3d xb_des = yb_des.cross(zb_des).normalized();

    // Compute the rotation matrix desidered
    Eigen::Matrix3d R_des;
    R_des.col(0) = xb_des;
    R_des.col(1) = yb_des;
    R_des.col(2) = zb_des;

    // Compute the rotation matrix error
    Eigen::Matrix3d Mat_e_rot = (R_des.transpose() * rot_matrix - rot_matrix.transpose() * R_des);

    Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
    Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

    thrust = (float)force_des.dot(rot_matrix.col(2).normalized());

    Vector3d outputs = -Kp_ang_mat_ * E_rot;

    acro[0] = outputs(0); // ROLL
    acro[1] = outputs(1); // PITCH
    acro[2] = outputs(2); // YAW

    return;
  };

  void DFController::computeYawSpeedControl(
      // Input
      const UAV_state &state,
      const float &yaw_speed_ref,
      const Vector3d &force_des,
      const double &dt,
      // Output
      Vector3d &acro,
      float &thrust)
  {
    tf2::Matrix3x3 m(state.rot);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    float yaw_angle_ref = yaw + yaw_speed_ref * dt;

    computeYawAngleControl(
        state,
        yaw_angle_ref,
        force_des,
        acro,
        thrust);

    return;
  };

  void DFController::updateGains_()
  {
    mass_ = parameters_["uav_mass"];
    antiwindup_cte_ = parameters_["antiwindup_cte"];
    alpha_ = parameters_["alpha"];

    position_Kp_lin_mat_ = Vector3d(
                               parameters_["position_following.position_Kp.x"],
                               parameters_["position_following.position_Kp.y"],
                               parameters_["position_following.position_Kp.z"])
                               .asDiagonal();

    position_Kd_lin_mat_ = Vector3d(
                               parameters_["position_following.position_Kd.x"],
                               parameters_["position_following.position_Kd.y"],
                               parameters_["position_following.position_Kd.z"])
                               .asDiagonal();

    position_Ki_lin_mat_ = Vector3d(
                               parameters_["position_following.position_Ki.x"],
                               parameters_["position_following.position_Ki.y"],
                               parameters_["position_following.position_Ki.z"])
                               .asDiagonal();

    traj_Kp_lin_mat_ = Vector3d(
                           parameters_["trajectory_following.position_Kp.x"],
                           parameters_["trajectory_following.position_Kp.y"],
                           parameters_["trajectory_following.position_Kp.z"])
                           .asDiagonal();

    traj_Ki_lin_mat_ = Vector3d(
                           parameters_["trajectory_following.position_Ki.x"],
                           parameters_["trajectory_following.position_Ki.y"],
                           parameters_["trajectory_following.position_Ki.z"])
                           .asDiagonal();

    traj_Kd_lin_mat_ = Vector3d(
                           parameters_["trajectory_following.position_Kd.x"],
                           parameters_["trajectory_following.position_Kd.y"],
                           parameters_["trajectory_following.position_Kd.z"])
                           .asDiagonal();

    velocity_Kp_lin_mat_ = Vector3d(
                               parameters_["speed_following.speed_Kp.x"],
                               parameters_["speed_following.speed_Kp.y"],
                               parameters_["speed_following.speed_Kp.z"])
                               .asDiagonal();

    velocity_Ki_lin_mat_ = Vector3d(
                               parameters_["speed_following.speed_Ki.x"],
                               parameters_["speed_following.speed_Ki.y"],
                               parameters_["speed_following.speed_Ki.z"])
                               .asDiagonal();

    velocity_Kd_lin_mat_ = Vector3d(
                               parameters_["speed_following.speed_Kd.x"],
                               parameters_["speed_following.speed_Kd.y"],
                               parameters_["speed_following.speed_Kd.z"])
                               .asDiagonal();

    Kp_ang_mat_ = Vector3d(
                      parameters_["angular_speed_controller.angular_gain.x"],
                      parameters_["angular_speed_controller.angular_gain.y"],
                      parameters_["angular_speed_controller.angular_gain.z"])
                      .asDiagonal();

    return;
  };
}
