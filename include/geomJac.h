#ifndef GEOMJAC_H
#define GEOMJAC_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <memory>

using namespace Eigen;
using namespace DQ_robotics;

MatrixXd geomJac(const DQ_SerialManipulator &robot, const MatrixXd &poseJacobian, const VectorXd &q, const int n);

#endif