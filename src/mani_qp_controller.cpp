// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "../include/mani_qp_controller.h"
#include <pluginlib/class_list_macros.h>
#include <cfloat>
#include <cmath>


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
  // joint_states_csv_ = load_csv("/home/gari/mani_check/src/mani_qp_controller/data/promp/q_position_mean_traj.csv");
  // joint_states_csv_ = load_csv("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/joint_position_traj_0912_1.csv");
  // joint_states_csv_ = load_csv("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/joint_position_demo.csv");
  // Input txt data (experiments data)
  std::string path = "/home/gari/mani_check/src/mani_qp_controller/data/bags/csv/joint_configurations_Drill_2.csv";
  joint_states_csv_ = load_csv(path);
  std::string joca_path = "/home/gari/icra2024_YuGongSilvaFigueredoPeper/data_output/geometrical_Jacobian_Drill_1.csv";
  std::cout<<"h1111111111111ol: " << std::endl;
  human_jacob = load_csv64(joca_path);
  std::cout<<"h22222222222222222222l: " << std::endl;
  // std::string wrist_path = "/home/gari/icra2024_YuGongSilvaFigueredoPeper/data_output/wrist_pose_Drill_2.csv";
  std::string wrist_path = "/home/gari/mani_check/src/mani_qp_controller/data/bags/csv/wrist_pose_shoulder_Drill_1.csv";

  wrist_pose = load_csv_8(wrist_path);
  //////////////////////////////////////////////////////
  col = wrist_pose.cols();
  // col = human_jacob.cols();

  // std::cout<<"human_jacobcol: " << human_jacob.col(0) <<std::endl;

  // mean_traj = load_csv_3("/home/gari/mani_check/src/mani_qp_controller/data/promp/xtrans_mean_traj.csv");
  // // mean_traj = load_csv_3("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/cartesian_translation.csv");
  // std::cout<<"mean_traj" << mean_traj.col(0) <<std::endl;

  // sigma_traj_3 = load_csv_3("/home/gari/mani_check/src/mani_qp_controller/data/promp/xtrans_sigma_traj_3.csv");
  // sigma_traj_3 = load_csv_3("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/cartesian_translation.csv");
  
  // // joint_states_csv_.resize(7,col);
  // std::cout<<"CSV file converted into Eigen::Matrix"<<std::endl;
  // std::cout<<"Matrix: \n"<<joint_states_csv_.col(0).transpose()<<std::endl;
  // std::cout<<"Matrix: \n"<<joint_states_csv_.col(1).transpose()<<std::endl;
  // std::cout<<"Matrix: \n"<<joint_states_csv_.col(2).transpose()<<std::endl;
  
  // q2xt
  // DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
  // std::cout<<"111111111111111111111111" <<std::endl;
  // x_t_traj_.resize(3,col);
  // xt_mean_dq_traj.resize(8,col);
  // std::cout<<"111222222222222222221" <<std::endl;

  // for(size_t i=0; i<col; i++){
  //     Eigen::Matrix<double, 7, 1> q = joint_states_csv_.col(i);
  //     Eigen::Matrix<double, 3, 1> xt_mean = mean_traj.col(i);
  //     std::cout<<"1113333333333333333333111111111" <<std::endl;
  //     Eigen::Matrix<double, 4, 1> trans;
  //     trans(0) = 0;
  //     trans.block(1,0,3,1) = xt_mean;
  //     DQ xt_mean_t = DQ(trans);
  //     std::cout<<"1144444444444444444444444444444411" <<std::endl;
  //     // forward kinematic model
  //     DQ xt = robot.fkm(q);
  //     Eigen::Matrix<double,3,1> xt_t = xt.translation().vec3();
  //     DQ xt_r = xt.rotation();
  //     DQ xt_mean_dq = xt_r + E_ * 0.5 * xt_mean_t * xt_r;
  //     Eigen::Matrix<double, 8, 1> xt_indiv = vec8(xt_mean_dq);
  //     xt_mean_dq_traj.col(i) = xt_indiv;
  //     x_t_traj_.col(i) = xt_t;
  //     std::cout<<"114444455555555555555555555444411" <<std::endl;
  // }
  // // x_t_traj = q2x(joint_states_csv_,col);
  // // x_t_traj = x_t_traj_;
  // std::cout<<"test x_t_traj: \n"<<x_t_traj_.col(1).transpose()<<std::endl;
  jaco_ten.resize(6, 8, col);
  for (int i=0; i<col; i++){
    Eigen::MatrixXd geom_huma;
    Eigen::Matrix<double, 6, 8> geom_J_hum;
    geom_huma = human_jacob.col(i);
    geom_huma.resize(8,8);
    geom_huma = geom_huma.transpose();
    geom_J_hum.block(0,0,3,8) = geom_huma.block(1,0,3,8);
    geom_J_hum.block(3,0,3,8) = geom_huma.block(5,0,3,8);
    array<DenseIndex, 3> offset = {0, 0, i};
    array<DenseIndex, 3> extent = {6, 8, 1};
    Tensor<double, 3> J_result_m2t = TensorMap<Tensor<double, 3>>(geom_J_hum.data(), 6, 8, 1);
    jaco_ten.slice(offset, extent) = J_result_m2t;
    DQ car_pose;
    car_pose = DQ(wrist_pose.col(i)).normalize();
    // Vector3d xt_mean;
    // // std::cout<<"car_posen: "<<car_pose<<std::endl;
    // // std::cout<<"crosbag_count: "<<rosbag_counter<<std::endl;
    
    // xt_mean = car_pose.translation().vec3();
    
    
    // bool judge;
    // double test;
    // test = wrist_pose.col(rosbag_counter)(0,0);
    // judge = std::isnan(test);
    // if (judge){
    //   wrist_last = wrist_pose;
    //   human_jacob_last = human_jacob;    
    // }
  }

  // bool write_xt;
  // MatrixXd save_tran;
  // write_xt = true;
  // save_tran = wrist_pose.transpose();
  // Matrix<double, 1, 3> save;

  // std::cout << "117777777777777777771lly" << std::endl;
  // std::ofstream file("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/wrist_translation.csv");    // Check if the file is open
  // if (write_xt){
  //   if (file.is_open()) {
  //       // Iterate through the matrix and write its elements to the CSV file
  //       for (int i = 0; i < save_tran.rows(); ++i) {
  //           std::cout << "11111111111111111111111lly" << std::endl;
  //           save = (DQ(save_tran.row(i)).normalize()).translation().vec3();
  //           std::cout << "12222222222222222222222y" << std::endl;
  //           for (int j = 0; j < 3; ++j) {
  //               std::cout << "save.cols()" << save.cols()<< std::endl;
  //               std::cout << "save(i, j))" << save(i, j) << i << j<< std::endl;
  //               file << save(i, j);
  //               //std::cout<<"save_Cartesian(i, j): "<<save_Cartesian(i, j) <<std::endl;
  //               // Add a comma and space except for the last element in each row
  //               if (j < save.cols() - 1) {
  //                 std::cout << "44444444444444444444442y" << std::endl;
  //                 std::cout << "save(i, j))" << save(i, j) << i << j<< std::endl;
  //                 file << ", ";
  //               }
  //           }
  //           // Add a newline character at the end of each row
  //           file << "\n";
  //       }        // Close the file
  //       file.close();
  //       std::cout << "Matrix written to save_Cartesian.csv successfully" << std::endl;
  //   } else {
  //       std::cerr << "Error opening the file." << std::endl;
  //   }
  //   abort();
  // }
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
  elapsed_time_ += period; // 34

  DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
  // std::cout<<"111111111111111111111111" <<std::endl;

  std::array<double, 7> q_array = robot_state.q;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());


  size_t rosbag_counter = i/10;
  Eigen::Matrix<double, 8, 1> q_desired = joint_states_csv_.col(i);
  // Eigen::Matrix<double, 3, 1> xt_mean = mean_traj.col(i);
  DQ car_pose;
  Matrix<double, 6, 8> geom_hu;

  array<DenseIndex, 3> offset = {0, 0, rosbag_counter};
  array<DenseIndex, 3> extent = {6, 8, 1};
  Tensor<double, 3> J_result_m2t;
  J_result_m2t = jaco_ten.slice(offset, extent);
  geom_hu = Map<Matrix<double,6,8>> (J_result_m2t.data(), 6, 8);
  Matrix<double, 8, 1> xt_human;
  // xt_human = wrist_pose.col(rosbag_counter);

  DQ xt = robot.fkm(q);
  DQ xt_tt = xt.translation();
  std::cout<<"Actual xt_t: "<<xt_tt<<std::endl;
  DQ xt_tr = xt.rotation();
  std::cout<<"Actual xt_r: "<<xt_tr<<std::endl;

  // reasselble the cartesian pose (from T & R) to 8x1 vector
  DQ xt_t = 0.001 * DQ(wrist_pose.col(rosbag_counter)).normalize().translation();
  std::cout<<"xt_t: "<<xt_t<<std::endl;
  DQ xt_r = 0.01*DQ(wrist_pose.col(rosbag_counter)).normalize().rotation();
  std::cout<<"xt_r: "<<xt_r<<std::endl;

  DQ xt_mean_dq = xt_r + E_ * 0.5 * xt_t * xt_r;
  
  car_pose = xt_mean_dq.normalize();
  xt_human = car_pose.vec8();
  Vector3d xt_mean;
  // std::cout<<"crosbag_count: "<<rosbag_counter<<std::endl;
  
  xt_mean = car_pose.translation().vec3();
  // std::cout<<"car_pose: "<<car_pose<<std::endl;
  // std::cout<<"xt_mean1: "<<xt_mean<<std::endl;
  // Eigen::Matrix<double, 4, 1> trans;
  // trans(0) = 0;
  // trans.block(1,0,3,1) = xt_mean;
  // DQ xt_mean_t = DQ(trans);
  // // forward kinematic model
  // DQ xt_r = car_pose.rotation();
  // DQ xt_mean_dq = xt_r + E_ * 0.5 * xt_mean_t * xt_r;
  // Eigen::Matrix<double, 8, 1> xt_indiv = vec8(car_pose); 

  // wrist_last = wrist_pose;
  // human_jacob_last = human_jacob;    
  // x_t_traj = q2x(joint_states_csv_,col);
  // x_t_traj = x_t_traj_;

  // double t0 = ros::Time::now().toSec();

  robot_state = state_handle_->getRobotState();

  // get joint position
  // get joint velocity
  std::array<double, 7> dq_array = robot_state.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
  // get end-effector pose (6x1) in base frame

  std::array<double, 6> dx_array = robot_state.O_dP_EE_c;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> dx(dx_array.data());
  DQ x_current = robot.fkm(q);
  MatrixXd x_c = x_current.translation().vec3();
  // std::cout<<"xt_mean2: "<<x_c<<std::endl;;
  // q_track.col(i) = q;

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
  }
  else{
    dx_fil = dx_fil_last + a * (dx - dx_fil_last);

  }
  dx_fil_last = dx_fil;
  // **************************************************************************

  // // get end-effenctor translation in base frame
  // Eigen::Matrix<double, 3, 1> x_EE_t = O_T_EE.block(0,3,3,1);
  // std::cout<<"Current eef translational position: "<<x_EE_t.transpose()<<std::endl;

  // Eigen::VectorXd dq_filtered_prev = dq_filtered;
  // dq_filtered = (1. - alpha_dq_filter) * dq_filtered + alpha_dq_filter * dq;

  // Eigen::Matrix<double, 7, 1> q_desired;
  // q_desired << -0.3, -0.5, -0.00208172, -2, -0.00172665, 1.57002, 0.794316;
  // std::cout<<"counter: "<<rosbag_counter<<std::endl;
  if (rosbag_counter >= col-1){
    ros::shutdown();
  }
  // q_desired = joint_states_csv_.col(rosbag_counter);
  Eigen::Matrix<double, 6, 1> x_desired;
  // x_desired.block(3,0,3,1) = sigma_traj_3.col(rosbag_counter); 
  Eigen::Matrix<double, 3, 1> Ide;
  Ide.setIdentity(); 
  x_desired.block(3,0,3,1) = Ide * 0.1; 
  x_desired.block(0,0,3,1) = xt_mean;
  Eigen::Matrix<double, 8, 1> xt_mean_full;
  xt_mean_full = car_pose.vec8();
  // std::cout<<"q_desired: "<<q_desired.transpose()<<std::endl;
  // std::cout<<"x_desired: "<<x_desired.transpose()<<std::endl;
  // Eigen::VectorXd dq_mod = qp_controller(q, dq, counter, q_desired, x_desired, xt_mean_full, dx_fil, dx_last, geom_hu);
  Eigen::VectorXd dq_mod = qp_controller(q, dq, counter, q_desired, x_desired, xt_human, dx_fil, dx_last, geom_hu);

  dx_last = dx_fil;
  // filter output dq_mod (joint velocity)
  if (i == 0){
    dq_fil = dq_mod;
  }
  else{
    dq_fil = dq_fil_last + a * (dq_mod - dq_fil_last);

  }
  dq_fil_last = dq_fil;
  // std::cout<<"Mani_command: "<<dq_mod.transpose()<<std::endl;
  // send command to robot
  // std::cout << "start loop" << std::endl;
  for (size_t i_ = 0; i_ < 7; ++i_) {
    velocity_joint_handles_[i_].setCommand(dq_fil(i_));
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
