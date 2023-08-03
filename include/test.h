#ifndef TEST_H
#define TEST_H

#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/jacobianEst.h"
#include "../include/geomJac.h"
// #include "../include/tmprod.h"
// #include "../include/symm2vec.h"
// #include "../include/spd2vec.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
// #include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <memory>
#include <array>
// #include "osqp/osqp.h"
#include "OsqpEigen/OsqpEigen.h"

using namespace DQ_robotics;

int test();

#endif