// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <array>

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <ros/package.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include "../include/qp_controller.h"
#include "../include/adm_controller.h"
#include "../include/save_data.h"
#include "convert_csv2matrix.h"
// #include "q2x.h"
#include <franka_hw/franka_cartesian_command_interface.h>

// #include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

namespace mani_qp_controller {

  class ManiQpController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::VelocityJointInterface,
                                            // hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
  public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;
    Eigen::Index counter;

  private:
    franka::Robot* robot_;
    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

    // hardware_interface::EffortJointInterface* effort_joint_interface_;
    // std::vector<hardware_interface::JointHandle> effort_joint_handles_;
    // std::vector<hardware_interface::JointHandle> joint_handles_;


    ros::Duration elapsed_time_;

    franka_hw::FrankaStateInterface *franka_state_interface_;
    franka_hw::FrankaModelInterface *model_interface_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

    franka::RobotState robot_state;

    Eigen::VectorXd dq_filtered;
    Eigen::VectorXd ddq_filtered;
    double alpha_dq_filter;
    double alpha_ddq_filter;
    bool filter_dq;

    // rotation limit
    Eigen::Matrix<double, 4, 4> O_T_EE_init;
    Eigen::Matrix<double, 3, 3> O_R_EE_init;
    Eigen::Matrix<double, 6, 1> F_ext_fil_last;
    Eigen::Matrix<double, 7, 1> dq_fil_last;
    Eigen::Matrix<double, 7, 1> tau_ext_fil_last;


    Eigen::MatrixXd q_track;
    size_t i;

    // matrix in which the data from csv go
    Eigen::MatrixXd joint_states_csv_;
    Eigen::MatrixXd x_t_traj_;
    size_t col;
    // Matrix<double,7,1> q_desired;
    // track or guid:
    bool tracking;   
    std::string adm_controller;
    Eigen::Matrix<double, 6, 1> dx_last;

    Eigen::MatrixXd save_Cartesian;
    //DQ_SerialManipulatorMDH robot;
  };

}  // namespace mani_qp_controller
