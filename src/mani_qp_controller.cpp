// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "../include/mani_qp_controller.h"
#include <pluginlib/class_list_macros.h>


namespace mani_qp_controller {

bool ManiQpController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "MANI_QP: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("MANI_QP: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("MANI_QP: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("MANI_QP: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "MANI_QP: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // try {
  //   auto state_handle = state_interface->getHandle(arm_id + "_robot");

  //   std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  //   for (size_t i = 0; i < q_start.size(); i++) {
  //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
  //       ROS_ERROR_STREAM(
  //           "MANI_QP: Robot is not in the expected starting position for "
  //           "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //           "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //       return false;
  //     }
  //   }
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "JointVelocityExampleController: Exception getting state handle: " << e.what());
  //   return false;
  // }
  
  // get robot initial joint positions and velocities
  franka_state_interface_ = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("MANI_QP: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));

  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("MANI_QP: Exception getting state handle: " << ex.what());
    return false;
  }
   
  // get robot state
  robot_state = state_handle_->getRobotState();

  // get joint position
  std::array<double, 7> q_array = robot_state.q;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  std::cout<<"Joint initial position: "<<q.transpose()<<std::endl;

  // get joint velocity
  std::array<double, 7> dq_array = robot_state.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  std::cout<<"Joint initial velocity: "<<dq.transpose()<<std::endl;

  // get end-effector pose in base frame
  std::array<double,16> O_T_EE_array = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE(O_T_EE_array.data());
  std::cout<<"Joint initial position: "<<std::endl<<O_T_EE<<std::endl;

  // get end-effenctor translation in base frame
  Eigen::Matrix<double, 3, 1> x_EE_t = O_T_EE.block(0,3,3,1);
  std::cout<<"Initial eef translational position: "<<x_EE_t.transpose()<<std::endl;

  // Eigen::VectorXd dq_mod = qp_controller(q);
  // std::cout<<"command: "<<dq_mod.transpose()<<std::endl;
  return true;
}

void ManiQpController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);

  // // get end effector pose
  // O_R_EE_init = robot.O_T_EE.block(0, 0, 3, 3);
  // O_T_EE_init = robot.O_T_EE;
}

void ManiQpController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  double t0 = ros::Time::now().toSec();

  robot_state = state_handle_->getRobotState();

  // get joint position
  std::array<double, 7> q_array = robot_state.q;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  // std::cout<<"Joint current position: "<<q.transpose()<<std::endl;

  // // get joint velocity
  // std::array<double, 7> dq_array = robot_state.dq;
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  // std::cout<<"Joint current velocity: "<<dq.transpose()<<std::endl;

  // // get end-effector pose in base frame
  // std::array<double,16> O_T_EE_array = robot_state.O_T_EE;
  // Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE(O_T_EE_array.data());
  // std::cout<<"Current position: "<<std::endl<<O_T_EE<<std::endl;

  // // get end-effenctor translation in base frame
  // Eigen::Matrix<double, 3, 1> x_EE_t = O_T_EE.block(0,3,3,1);
  // std::cout<<"Current eef translational position: "<<x_EE_t.transpose()<<std::endl;

  // Eigen::VectorXd dq_filtered_prev = dq_filtered;
  // dq_filtered = (1. - alpha_dq_filter) * dq_filtered + alpha_dq_filter * dq;

  // get joint acceleration
  // Eigen::VectorXd ddq(7);
  // ddq = (dq_filtered - dq_filtered_prev)/0.001;
  // std::cout<<"pass here--------------------------"<<std::endl;
  // qp_controller
  Eigen::VectorXd dq_mod = qp_controller(q);
  // std::cout<<"Mani_command: "<<dq_mod.transpose()<<std::endl;
  // send command to robot
  // std::cout << "start loop" << std::endl;
  for (size_t i = 0; i < 7; ++i) {
    velocity_joint_handles_[i].setCommand(dq_mod(i));
    
    // std::array<double, 7> dq_array_ = robot_state.dq;
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_(dq_array_.data());
    // std::cout<<"Joint current velocity: "<<dq_.transpose()<<std::endl;
  }
  double t1 = ros::Time::now().toSec();
  // std::cout<<"Update running time: "<<t1-t0<<std::endl;
  // ros::Duration time_max(8.0);
  // double omega_max = 0.1;
  // double cycle = std::floor(
  //     std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
  //                        time_max.toSec()));
  // double omega = cycle * omega_max / 2.0 *
  //                (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));
  // for (auto joint_handle : velocity_joint_handles_) {
  //   joint_handle.setCommand(omega);
  // // for (size_t i = 0; i < 7; ++i) {
  // //   velocity_joint_handles_[i].setCommand(omega);
  //   // std::cout<<"omega: "<<omega<<std::endl;
  //   std::array<double, 7> dq_array_ = robot_state.dq;
  //   Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_(dq_array_.data());
    // std::cout<<"Joint current velocity: "<<dq.transpose()<<std::endl;
    // }
  
}

void ManiQpController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(mani_qp_controller::ManiQpController,
                       controller_interface::ControllerBase)
