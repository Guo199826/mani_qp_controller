#ifndef JACOBIANESTVECTOR_H
#define JACOBIANESTVECTOR_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <functional>
#include "../include/geomJac.h"

using namespace Eigen;
using namespace DQ_robotics;

MatrixXd jacobianEstVector(const VectorXd& q, 
    const int n, const DQ_SerialManipulator &robot);

#endif