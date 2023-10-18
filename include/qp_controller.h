/**
(C) Copyright 2022 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
- Murilo Marques Marinho (murilo@g.ecc.u-tokyo.ac.jp)

Instructions:
Prerequisites:
- dqrobotics
- dqrobotics-interface-vrep

1) Open the CoppeliaSim scene joint_velocity_commands.ttt
2) Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
3) Compile, run and enjoy!
*/
#ifndef QP_CONTROLLER_H
#define QP_CONTROLLER_H
#include <iostream>
#include <dqrobotics/DQ.h>
// #include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include "../include/FrankaRobot.h"
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "../include/dq2tfm.h"
#include "../include/logmap.h"
#include "../include/jacobianEst.h"
#include "../include/jacobianEstVector.h"
#include "../include/geomJac.h"
#include "../include/manipulabilityJacobian.h"
#include "../include/mani_qp_controller.h"
// #include "../include/franka_analytical_ik-main/franka_ik_He.hpp"
// #include "osqp/osqp.h"
#include "OsqpEigen/OsqpEigen.h"
#include <chrono>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>

using namespace Eigen;
using namespace DQ_robotics;

VectorXd qp_controller(const Matrix<double,7,1> &q_, const Matrix<double,7,1> &dq_, 
                        Index &counter, const Matrix<double,7,1> &q_desired,
                        const Matrix<double,6,1> &x_desired,
                        const Matrix<double, 8, 1> &xt_mean,
                        const Matrix<double,6,1> &dx,
                        const Matrix<double,6,1> &dx_last,
                        const std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle_);

#endif