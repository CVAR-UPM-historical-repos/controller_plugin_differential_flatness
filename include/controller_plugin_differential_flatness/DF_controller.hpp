
#ifndef __DF_CONTROLLER_H__
#define __DF_CONTROLLER_H__

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unordered_map>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace differential_flatness_controller
{
  using Vector3d = Eigen::Vector3d;

  struct UAV_state
  {
    Vector3d pos;
    Vector3d vel;
    tf2::Quaternion rot;
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
    Eigen::Vector3d getPositionError();
    Eigen::Vector3d getVelocityError();
    Eigen::Vector3d getTrajPositionError();
    void resetError();

    bool setParameter(const std::string &param, const double &value);
    bool getParameter(const std::string &param, double &value);
    bool isParameter(const std::string &param);
    bool setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list);
    std::vector<std::pair<std::string, double>> getParametersList();

    Vector3d computePositionControl(
        const UAV_state &state_,
        const Control_ref &ref_,
        const double &dt,
        const Vector3d &speed_limit,
        const bool &proportional_limitation);

    Vector3d computeVelocityControl(
        const UAV_state &state_,
        const Control_ref &ref_,
        const double &dt);

    Vector3d computeTrajectoryControl(
        const UAV_state &state,
        const Control_ref &ref,
        const double &dt);

    void computeYawAngleControl(
        // Input
        const UAV_state &state,
        const float &yaw_angle_ref,
        const Vector3d &force_des,
        // Output
        Vector3d &acro,
        float &thrust);

    void computeYawSpeedControl(
        // Input
        const UAV_state &state,
        const float &yaw_speed_ref,
        const Vector3d &force_des,
        const double &dt,
        // Output
        Vector3d &acro,
        float &thrust);

  private:
    UAV_state state_;
    Control_ref ref_;

    Eigen::Vector3d position_accum_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_accum_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d traj_position_accum_error_ = Eigen::Vector3d::Zero();

    const float g = 9.81f;
    const Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, g);

    std::unordered_map<std::string, double> parameters_ = {
        {"mass", 0.75},
        {"antiwindup_cte", 5.0},
        {"alpha", 0.1},
        {"position_following.position_Kp.x", 2.0},
        {"position_following.position_Kp.y", 2.0},
        {"position_following.position_Kp.z", 2.5},
        {"position_following.position_Kd.x", 0.0},
        {"position_following.position_Kd.y", 0.0},
        {"position_following.position_Kd.z", 0.0},
        {"position_following.position_Ki.x", 0.01},
        {"position_following.position_Ki.y", 0.01},
        {"position_following.position_Ki.z", 0.11},
        {"speed_following.speed_Kp.x", 2.8},
        {"speed_following.speed_Kp.y", 2.8},
        {"speed_following.speed_Kp.z", 4.0},
        {"speed_following.speed_Kd.x", 0.008},
        {"speed_following.speed_Kd.y", 0.008},
        {"speed_following.speed_Kd.z", 0.0},
        {"speed_following.speed_Ki.x", 1.6},
        {"speed_following.speed_Ki.y", 1.6},
        {"speed_following.speed_Ki.z", 0.1},
        {"trajectory_following.position_Kp.x", 6.0},
        {"trajectory_following.position_Kp.y", 6.0},
        {"trajectory_following.position_Kp.z", 6.0},
        {"trajectory_following.position_Kd.x", 2.5},
        {"trajectory_following.position_Kd.y", 2.5},
        {"trajectory_following.position_Kd.z", 3.0},
        {"trajectory_following.position_Ki.x", 0.05},
        {"trajectory_following.position_Ki.y", 0.05},
        {"trajectory_following.position_Ki.z", 0.065},
        {"angular_speed_controller.angular_gain.x", 5.5},
        {"angular_speed_controller.angular_gain.y", 5.5},
        {"angular_speed_controller.angular_gain.z", 2.0},
    };

    Eigen::Matrix3d position_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d position_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d position_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

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
