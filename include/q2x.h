#ifndef Q2X_H
#define Q2X_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include "../include/FrankaRobot.h"

using namespace Eigen;
using namespace DQ_robotics;

MatrixXd q2x(const MatrixXd &q_traj, const size_t col);

#endif