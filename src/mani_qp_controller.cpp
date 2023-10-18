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

  //////////////////////////////////////////////
  // get parameters from yaml file
  // if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
  //   ROS_ERROR("IEC:  Invalid or no k_gain parameters provided, aborting "
  //             "controller init!");
  //   return false;
  // }

  // // populate damping and stiffness matrices
  // K.setZero();
  // D.setZero();

  // for (int i = 0; i < 7; i++) {
  //   K(i, i) = k_gains_[i];
  //   D(i, i) = 1.3 * sqrt(k_gains_[i]);
  // }
  //////////////////////////////////////////////

  // get robot state
  robot_state = state_handle_->getRobotState();

  // // get joint position
  // std::array<double, 7> q_array = robot_state.q;
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
  // std::cout<<"Joint initial position: "<<q.transpose()<<std::endl;

  // // get joint velocity
  // std::array<double, 7> dq_array = robot_state.dq;
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  // std::cout<<"Joint initial velocity: "<<dq.transpose()<<std::endl;

  // // get end-effector pose in base frame
  // std::array<double,16> O_T_EE_array = robot_state.O_T_EE;
  // Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE(O_T_EE_array.data());
  // std::cout<<"Joint initial position: "<<std::endl<<O_T_EE<<std::endl;

  // // get end-effenctor translation in base frame
  // Eigen::Matrix<double, 3, 1> x_EE_t = O_T_EE.block(0,3,3,1);
  // std::cout<<"Initial eef translational position: "<<x_EE_t.transpose()<<std::endl;

  i = 0;
  counter = 0;
  F_ext_fil_last.setZero();
  dx_last.setZero();

  // get joint angle trajectory from csv
  joint_states_csv_ = load_csv("/home/gari/mani_check_before/src/mani_qp_controller/data/promp/q_position_mean_traj.csv");
  // joint_states_csv_ = load_csv("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/joint_position_traj_0912_1.csv");
  // joint_states_csv_ = load_csv("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/joint_position_demo.csv");
  // Input txt data (experiments data)
  // std::string path = "/home/gari/mani_check/src/mani_qp_controller/data/bags/csv/joint_configurations_Drill_1.csv";
  // joint_states_csv_ = load_csv(path);
  //////////////////////////////////////////////////////
  col = joint_states_csv_.cols();
  std::cout<<"col: " << col <<std::endl;


  mean_traj = load_csv_3("/home/gari/mani_check_before/src/mani_qp_controller/data/promp/xtrans_mean_traj.csv");
  // mean_traj = load_csv_3("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/cartesian_translation.csv");
  std::cout<<"mean_traj" << mean_traj.col(0) <<std::endl;

  sigma_traj_3 = load_csv_3("/home/gari/mani_check_before/src/mani_qp_controller/data/promp/xtrans_sigma_traj_3.csv");
  // sigma_traj_3 = load_csv_3("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/cartesian_translation.csv");
  
  // // joint_states_csv_.resize(7,col);
  // std::cout<<"CSV file converted into Eigen::Matrix"<<std::endl;
  // std::cout<<"Matrix: \n"<<joint_states_csv_.col(0).transpose()<<std::endl;
  // std::cout<<"Matrix: \n"<<joint_states_csv_.col(1).transpose()<<std::endl;
  // std::cout<<"Matrix: \n"<<joint_states_csv_.col(2).transpose()<<std::endl;
  
  // q2xt
  DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
  x_t_traj_.resize(3,col);
  xt_mean_dq_traj.resize(8,col);

  for(size_t i=0; i<col; i++){
      Eigen::Matrix<double, 7, 1> q = joint_states_csv_.col(i);
      Eigen::Matrix<double, 3, 1> xt_mean = mean_traj.col(i);
      Eigen::Matrix<double, 4, 1> trans;
      trans(0) = 0;
      trans.block(1,0,3,1) = xt_mean;
      DQ xt_mean_t = DQ(trans);
      // std::cout<<"q assignmnt"<< q <<std::endl;
      // forward kinematic model
      DQ xt = robot.fkm(q);
      Eigen::Matrix<double,3,1> xt_t = xt.translation().vec3();
      DQ xt_r = xt.rotation();
      DQ xt_mean_dq = xt_r + E_ * 0.5 * xt_mean_t * xt_r;
      Eigen::Matrix<double, 8, 1> xt_indiv = vec8(xt_mean_dq);
      xt_mean_dq_traj.col(i) = xt_indiv;
      x_t_traj_.col(i) = xt_t;
  }
  // x_t_traj = q2x(joint_states_csv_,col);
  // x_t_traj = x_t_traj_;
  std::cout<<"test x_t_traj: \n"<<x_t_traj_.col(1).transpose()<<std::endl;
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
  // get joint velocity
  std::array<double, 7> dq_array = robot_state.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  // get end-effector pose (6x1) in base frame
  std::array<double, 6> dx_array = robot_state.O_dP_EE_c;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> dx(dx_array.data());
  // q_track.col(i) = q;

  // filtered dq_output
  Matrix<double, 7, 1> dq_fil_output;
  // filtered dq
  Matrix<double, 7, 1> dq_fil;
  // filtered dx
  Matrix<double, 6, 1> dx_fil;

  // ******************* Contributor: Yuhe Gong ******************************
  // Low Pass Filter
  // T_s: sampling time = 0.001 
  double Ts = 0.001;
  // F_c: cut_off frequency = 1
  double fc = 0.5;
  // a = T_s / (T_s + RC) = 2 * pi * f_c * T_s / (2 * pi * f_c * T_s + 1)
  double a = 2 * 3.14 * fc * Ts / (2 * 3.14 * fc * Ts + 1);
  // Low Pass Filter: y_n = a x_n + (1 - a) * y_{n-1} = y_{n-1} + a * (x_n - y_{n-1})
  if (i == 0){
    dx_fil = dx;
    dq_fil = dq;
  }
  else{
    dx_fil = dx_fil_last + a * (dx - dx_fil_last);
    dq_fil = dq_fil_last + a * (dq - dq_fil_last);
  }
  dx_fil_last = dx_fil;
  dq_fil_last = dq_fil;
  // **************************************************************************

  // // get end-effenctor translation in base frame
  // Eigen::Matrix<double, 3, 1> x_EE_t = O_T_EE.block(0,3,3,1);
  // std::cout<<"Current eef translational position: "<<x_EE_t.transpose()<<std::endl;

  // Eigen::VectorXd dq_filtered_prev = dq_filtered;
  // dq_filtered = (1. - alpha_dq_filter) * dq_filtered + alpha_dq_filter * dq;

  Eigen::Matrix<double, 7, 1> q_desired;
  // q_desired << -0.3, -0.5, -0.00208172, -2, -0.00172665, 1.57002, 0.794316;
  size_t rosbag_counter = i/10; // 34
  // std::cout<<"counter: "<<rosbag_counter<<std::endl;
  if (rosbag_counter >= col-1){
    ros::shutdown();
  }
  q_desired = joint_states_csv_.col(rosbag_counter);
  Eigen::Matrix<double, 6, 1> x_desired;
  // For Experiment 1: add cartesian position offset
  Eigen::Matrix<double, 3, 1> offset_x;
  offset_x<<0, 0.2, 0.1;
  x_desired.block(0,0,3,1) = mean_traj.col(rosbag_counter) + offset_x; 
  x_desired.block(3,0,3,1) = sigma_traj_3.col(rosbag_counter); 
  Eigen::Matrix<double, 8, 1> xt_mean_full;
  xt_mean_full = xt_mean_dq_traj.col(rosbag_counter);
  // std::cout<<"q_desired: "<<q_desired.transpose()<<std::endl;
  // std::cout<<"x_desired: "<<x_desired.transpose()<<std::endl;

  Eigen::VectorXd dq_mod = qp_controller(q, dq_fil, counter, q_desired, x_desired, xt_mean_full, dx_fil, dx_last, model_handle_);
  dx_last = dx_fil;
  // filter output dq_mod (joint velocity)
  if (i == 0){
    dq_fil_output = dq_mod;
  }
  else{
    dq_fil_output = dq_fil_last_output + a * (dq_mod - dq_fil_last_output);

  }
  dq_fil_last_output = dq_fil_output;
  // std::cout<<"Mani_command: "<<dq_mod.transpose()<<std::endl;
  // send command to robot
  for (size_t i_ = 0; i_ < 7; ++i_) {
    velocity_joint_handles_[i_].setCommand(dq_fil_output(i_));
    // std::cout<<"Joint current velocity: "<<dq_.transpose()<<std::endl;
  }

  i++;
}

void ManiQpController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(mani_qp_controller::ManiQpController,
                       controller_interface::ControllerBase)
