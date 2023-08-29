#ifndef ADM_CONTROLLER_H
#define ADM_CONTROLLER_H

#include <iostream>
#include <dqrobotics/DQ.h>
#include "../include/FrankaRobot.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "geomJac.h"

using namespace Eigen;
using namespace DQ_robotics;

VectorXd tran_adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext);
VectorXd rot_adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext);
VectorXd full_adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext);

#endif