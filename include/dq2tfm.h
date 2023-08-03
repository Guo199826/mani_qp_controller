#ifndef DQ2TFM_H
#define DQ2TFM_H

#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace DQ_robotics;
using namespace Eigen;

Matrix4d dq2tfm(const DQ &dq);

#endif