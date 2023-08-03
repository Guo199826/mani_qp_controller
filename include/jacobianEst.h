#ifndef JACOBIANEST_H
#define JACOBIANEST_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <functional>

using namespace Eigen;
using namespace DQ_robotics;

Tensor<double, 3> jacobianEst(std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac, const VectorXd& q, const int n,
    const DQ_SerialManipulator &robot);

#endif