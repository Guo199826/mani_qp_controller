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
  
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
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

  i = 0;
  counter = 0;
  F_ext_fil_last.setZero();

  // get joint angle trajectory from csv
  joint_states_csv_ = load_csv("/home/gari/mani_qp_ws_for_traj/src/mani_qp_controller/data/csv/joint_position_exam_force.csv");
  // joint_states_csv = joint_states_csv_;
  col = joint_states_csv_.cols();
  std::cout<<"col of matrixXd: "<<col<<std::endl;

  // joint_states_csv_.resize(7,col);
  std::cout<<"CSV file converted into Eigen::Matrix"<<std::endl;
  std::cout<<"Matrix: \n"<<joint_states_csv_.col(0).transpose()<<std::endl;
  std::cout<<"Matrix: \n"<<joint_states_csv_.col(1).transpose()<<std::endl;
  std::cout<<"Matrix: \n"<<joint_states_csv_.col(2).transpose()<<std::endl;
  
  // q2xt
  DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
  x_t_traj_.resize(3,col);
  for(size_t i=0; i<col; i++){
      Eigen::Matrix<double,7,1> q = joint_states_csv_.col(i);
      // std::cout<<"q assignmnt"<< q <<std::endl;
      // forward kinematic model
      DQ xt = robot.fkm(q);
      Eigen::Matrix<double,3,1> xt_t = xt.translation().vec3();
      x_t_traj_.col(i) = xt_t;
  }
  // x_t_traj = q2x(joint_states_csv_,col);
  // x_t_traj = x_t_traj_;
  std::cout<<"test x_t_traj: \n"<<x_t_traj_.col(col-1).transpose()<<std::endl;
  
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

  // double t0 = ros::Time::now().toSec();

  robot_state = state_handle_->getRobotState();

  // get joint position
  std::array<double, 7> q_array = robot_state.q;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  
  // q_track.col(i) = q;

  // get external force & torque
  std::array<double, 6> F_ext_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 3, 2>> F_ext_6D(F_ext_array.data());
  Matrix<double, 3, 1> F_ext = F_ext_6D.col(0);
  Matrix<double, 3, 1> F_ext_fil;
 
  // ******************* Contributor: Yuhe Gong ******************************
  // Low Pass Filter
  // T_s: sampling time = 0.001 
  double Ts = 0.001;
  // F_c: cut_off frequency = 1
  double fc = 1;
  // a = T_s / (T_s + RC) = 2 * pi * f_c * T_s / (2 * pi * f_c * T_s + 1)
  double a = 2 * 3.14 * fc * Ts / (2 * 3.14 * fc * Ts + 1);
  // Low Pass Filter: y_n = a x_n + (1 - a) * y_{n-1} = y_{n-1} + a * (x_n - y_{n-1})
  if (i == 0){
    F_ext_fil = F_ext;
  }
  else{
    F_ext_fil = F_ext_fil_last + a * (F_ext - F_ext_fil_last);
  }
  F_ext_fil_last = F_ext_fil;
  // **************************************************************************

  // ROS_INFO_STREAM("Joint current position: "<<q.transpose());

  // get joint velocity
  std::array<double, 7> dq_array = robot_state.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  // ROS_INFO_STREAM("Joint current velocity: "<<dq.transpose());

  // get end-effector pose in base frame
  // std::array<double,16> O_T_EE_array = robot_state.O_T_EE;
  // Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE(O_T_EE_array.data());
  // ROS_INFO_STREAM("Current position: \n"<<O_T_EE);

  // // get end-effenctor translation in base frame
  // Eigen::Matrix<double, 3, 1> x_EE_t = O_T_EE.block(0,3,3,1);
  // std::cout<<"Current eef translational position: "<<x_EE_t.transpose()<<std::endl;

  // Eigen::VectorXd dq_filtered_prev = dq_filtered;
  // dq_filtered = (1. - alpha_dq_filter) * dq_filtered + alpha_dq_filter * dq;

  // get joint acceleration
  // Eigen::VectorXd ddq(7);
  // ddq = (dq_filtered - dq_filtered_prev)/0.001;

  Eigen::Matrix<double, 7, 1> q_desired;
  // q_desired << -0.3, -0.5, -0.00208172, -2, -0.00172665, 1.57002, 0.794316;
  size_t rosbag_counter = i/10; // 34
  // std::cout<<"counter: "<<rosbag_counter<<std::endl;
  q_desired = joint_states_csv_.col(rosbag_counter);
  Eigen::Matrix<double, 3, 1> x_desired = x_t_traj_.col(rosbag_counter); 
  // std::cout<<"q_desired: "<<q_desired.transpose()<<std::endl;
  // std::cout<<"x_desired: "<<x_desired.transpose()<<std::endl;
  Eigen::VectorXd dq_mod;
  if(tracking){
    dq_mod = qp_controller(q, dq, counter, q_desired, x_desired);
    if (rosbag_counter >= col-1){
      ros::shutdown();
    }
  } else{
    dq_mod = adm_controller(q, F_ext_fil);
  }
  // std::cout<<"Mani_command: "<<dq_mod.transpose()<<std::endl;
  // send command to robot
  // std::cout << "start loop" << std::endl;
  for (size_t i_ = 0; i_ < 7; ++i_) {
    velocity_joint_handles_[i_].setCommand(dq_mod(i_));
    // std::array<double, 7> dq_array_ = robot_state.dq;
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_(dq_array_.data());
    // std::cout<<"Joint current velocity: "<<dq_.transpose()<<std::endl;
  }
  // double t1 = ros::Time::now().toSec();
  // ROS_INFO_STREAM("Update running time: "<< t1-t0);
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
  i++;
  
}

void ManiQpController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  std::cout<<"stopping() running... "<<std::endl;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(mani_qp_controller::ManiQpController,
                       controller_interface::ControllerBase)
