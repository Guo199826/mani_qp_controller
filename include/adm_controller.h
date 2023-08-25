#ifndef ADM_CONTROLLER_H
#define ADM_CONTROLLER_H

#include <iostream>
#include <dqrobotics/DQ.h>
#include "../include/FrankaRobot.h"
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "geomJac.h"

using namespace Eigen;
using namespace DQ_robotics;

VectorXd adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext);

#endif