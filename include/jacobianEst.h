#ifndef JACOBIANEST_H
#define JACOBIANEST_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include "../include/mani_qp_controller.h"
// #include <functional>
#include "../include/geomJac.h"
#include "../include/franka_model.h"

using namespace Eigen;
using namespace DQ_robotics;

// for velocity Manip.
Tensor<double, 3> jacobianEst(const VectorXd& q, const int n,
    const DQ_SerialManipulator &robot);

// derivative of Mass matrix wrt joint position
Tensor<double, 3> jacobianEstMass(const Matrix<double, 7, 1>& q, const int n,
    const DQ_SerialManipulator &robot);

//for dyn. Manip.
Tensor<double, 3> jacobianEstDynManip(const Matrix<double, 7, 1>& q, const int n,
    const DQ_SerialManipulator &robot,
    const std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle,
    const std::array<double, 9> &total_inertia,
    const double total_mass,
    const std::array<double, 3> &F_x_Ctotal);
    
#endif