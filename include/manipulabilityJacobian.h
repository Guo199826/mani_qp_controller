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

Tensor<double, 3> manipulabilityJacobianDyn(const MatrixXd &L_ct, const Tensor<double, 3> &J_grad,
                    const Tensor<double, 3> &mass_diff_, const Matrix<double, 7, 7> &Mass);

MatrixXd redManipulabilityJacobian(const MatrixXd & geomJ_, const Tensor<double, 3> &J_grad_);
MatrixXd redManipulabilityJacobianDyn(const MatrixXd &L_ct, const Tensor<double, 3> &J_grad,
                    const Tensor<double, 3> &mass_diff_, const Matrix<double, 7, 7> &Mass);


#endif