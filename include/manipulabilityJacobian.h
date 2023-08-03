#ifndef MANIPULABILITYJACOBIAN_H
#define MANIPULABILITYJACOBIAN_H
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include "../include/spdToVec.h"
#include "../include/tmprod.h"
#include "../include/readData.h"

using namespace Eigen;
using namespace DQ_robotics;

Tensor<double, 3> manipulabilityJacobian(const MatrixXd & geomJ, const Tensor<double, 3> &J_grad);

MatrixXd redManipulabilityJacobian(const MatrixXd & geomJ_, const Tensor<double, 3> &J_grad_);


#endif