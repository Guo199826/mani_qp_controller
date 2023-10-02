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

  // auto* effort_joint_interface_ = robot_hardware->get<hardware_interface::EffortJointInterface>();
  // if (effort_joint_interface_ == nullptr) {
  //   ROS_ERROR_STREAM("ManiQPController: Error getting effort joint interface from hardware");
  //   return false;
  // }
  // for (size_t i = 0; i < 7; ++i) {
  //   try {
  //     joint_handles_.push_back(effort_joint_interface_->getHandle(joint_names[i]));
  //   } catch (const hardware_interface::HardwareInterfaceException& ex) {
  //     ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
  //     return false;
  //   }
  // }

  
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
  std::string path ="/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/joint_position_exp3.csv";
  // joint_states_csv_ = load_csv("/home/gari/mani_check/src/mani_qp_controller/data/promp/q_position_mean_traj.csv");
  joint_states_csv_ = load_csv(path);
  // joint_states_csv = joint_states_csv_;
  col = joint_states_csv_.cols();
  std::cout<<"col of matrixXd: "<<col<<std::endl;

  // joint_states_csv_.resize(7,col);
  std::cout<<"CSV file converted into Eigen::Matrix"<<std::endl;
  std::cout<<"Matrix: \n"<<joint_states_csv_.col(0).transpose()<<std::endl;
  std::cout<<"Matrix: \n"<<joint_states_csv_.col(1).transpose()<<std::endl;
  std::cout<<"Matrix: \n"<<joint_states_csv_.col(2).transpose()<<std::endl;
  
  MatrixXd save_Cartesian;
  MatrixXd save_Rotation;
  save_Cartesian.setZero();
  save_Rotation.setZero();
  MatrixXd save_tran;
  MatrixXd save_rot;
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
      Eigen::Matrix<double, 6, 1> xt_6;
      Eigen::Vector4d xt_r = xt.rotation().vec4();
      Eigen::Matrix<double, 4, 1> xr_combine;
      // xr_combine.block(0,0,1,1) = rotation_angle(xt);
      // xr_combine.block(1,0,3,1) = xt.rotation_axis();

      // if (i==0){
      //   save_Cartesian = xt_t;
      //   save_Rotation = xt_r;
      // }else {
      //   save_Cartesian.conservativeResize(3, save_Cartesian.cols()+1);
      //   save_Cartesian.col(save_Cartesian.cols()-1) = xt_t;
      // }
  }
      
      // Eigen::Quaterniond rotationQuaternion(xt_r(0), xt_r(1), xt_r(2), xt_r(3));
      // // Convert the rotation quaternion into a 3x3 rotation matrix
      // Eigen::Matrix3d rotationMatrix = rotationQuaternion.toRotationMatrix();
      // Eigen::Vector3d Euler = rotationMatrix.eulerAngles(2,1,0);
      // double roll, pitch, yaw;
      // roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
      // pitch = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)));
      // yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
      // roll = Euler(0);
      // pitch = Euler(1);
      // yaw = Euler(2);
      // xt_6(0,0)= roll;
      // xt_6(1,0)= pitch;
      // xt_6(2,0)= yaw;
      // xt_6.block(3,0,3,1) = xt_t;
      // x_t_traj_.col(i) = xt_6;

    // Open a CSV file for writing
    // bool write_xt;
    // write_xt = false;
    // save_tran = save_Cartesian.transpose();
    // std::ofstream file("/home/gari/mani_tracking_test/src/mani_qp_controller/data/csv/cartesian_translation03.csv");    // Check if the file is open
    // if (write_xt){
    //   if (file.is_open()) {
    //       // Iterate through the matrix and write its elements to the CSV file
    //       for (int i = 0; i < save_tran.rows(); ++i) {
    //           for (int j = 0; j < save_tran.cols(); ++j) {
    //               file << save_tran(i, j);
    //               //std::cout<<"save_Cartesian(i, j): "<<save_Cartesian(i, j) <<std::endl;
    //               // Add a comma and space except for the last element in each row
    //               if (j < save_tran.cols() - 1) {
    //                   file << ", ";
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
  // x_t_traj = q2x(joint_states_csv_,col);
  // x_t_traj = x_t_traj_;
  // Eigen::Matrix<double, 3, 1> mi;
  // mi.setZero();
  // save_Cartesian = mi;
  // std::cout<<"test x_t_traj: \n"<<x_t_traj_.col(col-1).transpose()<<std::endl;
  
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
  DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
  DQ xt = robot.fkm(q);
  Eigen::Matrix<double, 3, 1> xt_t = xt.translation().vec3();

  // filtered dq
  Eigen::Matrix<double, 7, 1> dq_fil;
  // q_track.col(i) = q;

  // get external force & torque
  std::array<double, 6> F_ext_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 3, 2>> F_ext_6D(F_ext_array.data());
  Matrix<double, 6, 1> F_ext;
  F_ext.block(0,0,3,1)= F_ext_6D.col(1); // get torque
  F_ext.block(3,0,3,1)= F_ext_6D.col(0); // get force
  MatrixXd F_ext_fil;
  MatrixXd tau_ext_fil;
  double F_ext_threshold = 5;
  std::array<double, 7> gravity_array = model_handle_->getGravity();;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> Gravity(gravity_array.data());
  std::array<double, 7> tau_ext_array = robot_state.tau_J;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext(tau_ext_array.data());
  tau_ext = tau_ext - Gravity;

  if ((F_ext.col(0)).norm() > F_ext_threshold){
      F_ext = ((F_ext.col(0)).norm() - F_ext_threshold) / (F_ext.col(0)).norm() * F_ext;
  }
  else{
    F_ext.setZero();
  }
  if ((tau_ext.col(0)).norm() > F_ext_threshold){
      tau_ext = ((tau_ext.col(0)).norm() - F_ext_threshold) / (tau_ext.col(0)).norm() * tau_ext;
  }
  else{
    tau_ext.setZero();
  }

 
  // ******************* Contributor: Yuhe Gong ******************************
  // Low Pass Filter: y_n = a x_n + (1 - a) * y_{n-1} = y_{n-1} + a * (x_n - y_{n-1})
  // T_s: sampling time = 0.001 
  // F_c: cut_off frequency = 1
  // a = T_s / (T_s + RC) = 2 * pi * f_c * T_s / (2 * pi * f_c * T_s + 1)
  double Ts = 0.001;
  double fc = 0.1;
  double a = 2 * 3.14 * fc * Ts / (2 * 3.14 * fc * Ts + 1);
  if (i == 0){
    F_ext_fil = F_ext;
    tau_ext_fil = tau_ext;
    //F_ext_fil = tau_ext_hat_filtered;
    F_ext_fil_last.setZero();
    tau_ext_fil_last.setZero();
  }
  else{
    F_ext_fil = F_ext_fil_last + a * (F_ext - F_ext_fil_last);
    tau_ext_fil = tau_ext_fil_last + a * (tau_ext - tau_ext_fil_last);
  }
  // Integral on F_ext_fil
  Eigen::MatrixXd F_Inter;
  Eigen::MatrixXd tau_Inter;
  // std::cout<<"Measured F: "<<F_ext_fil.transpose()<<std::endl;
  F_Inter = F_ext_fil + 0.1 * F_ext_fil_last;
  tau_Inter = tau_ext_fil + 0.1 * tau_ext_fil_last;

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

  // get end-effector pose (6x1) in base frame
  std::array<double, 6> dx_array = robot_state.O_dP_EE_c;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> dx(dx_array.data());

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
  size_t rosbag_counter = i/10;
  // size_t rosbag_counter = i/1000;

  // std::cout<<"counter: "<<rosbag_counter<<std::endl;
  q_desired = joint_states_csv_.col(rosbag_counter);
  Eigen::Matrix<double, 3, 1> x_desired = x_t_traj_.col(rosbag_counter); 
  // std::cout<<"q_desired: "<<q_desired.transpose()<<std::endl;
  // std::cout<<"x_desired: "<<x_desired.transpose()<<std::endl;
  Eigen::VectorXd dq_mod;
  
  std::array<double, 42> J_array = model_handle_->getZeroJacobian(franka::Frame::kStiffness);
  Eigen::Matrix<double, 6, 7> J_franka = Eigen::Map<Eigen::Matrix<double, 6, 7>> (J_array.data());
  Eigen::Matrix<double, 6, 7> J_franka_;
  J_franka_.block(0,0,3,7) = J_franka.block(3,0,3,7);
  J_franka_.block(3,0,3,7) = J_franka.block(0,0,3,7);
  // std::cout<<"J_franka: "<< std::endl<<J_franka_<<std::endl;

  dq_mod = qp_controller(q, dq, counter, q_desired, x_desired, F_Inter, dx, dx_last, tau_Inter);
  F_ext_fil_last = F_ext_fil;
  tau_ext_fil_last = tau_ext_fil;
  dx_last = dx;
  // std::cout<<"Mani_command: "<<dq_mod.transpose()<<std::endl;
  // if (rosbag_counter >= col-1){      
  //   ros::shutdown();
  // }
  for (size_t i_ = 0; i_ < 7; ++i_) {
    velocity_joint_handles_[i_].setCommand(dq_mod(i_));
  }

  // for (size_t i = 0; i < 7; ++i) {
  //   joint_handles_[i].setCommand(dq_mod(i));
  //   }

  // std::cout<<"Joint current velocity: "<<dq_mod.transpose()<<std::endl;

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
  //   std::cout<<"Joint current velocity: "<<dq.transpose()<<std::endl;
  //   }
  i++;
  
}

void ManiQpController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  // std::cout<<"stopping() running... "<<std::endl;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(mani_qp_controller::ManiQpController,
                       controller_interface::ControllerBase)
