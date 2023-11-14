/**
Contributors to this file:
    Yuhe Gong - yuhe.gong@nottingham.ac.uk

File:
    Input: the real sense shoulder joints q1, q2, q3
           elbow flexion joint: q4
           zero joint: q5, q6, the wrist joint always set 0
    Base frame/ Reference frame: same as Franka Emika Robots

*/
#ifndef DQ_ROBOTS_REALSENSEROBOT_H
#define DQ_ROBOTS_REALSENSEROBOT_H
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

class RealSenseRobot
{
public:
    static MatrixXd _get_mdh_matrix();
    static DQ _get_offset_base();
    static DQ _get_offset_flange();
    static std::tuple<const VectorXd, const VectorXd> _get_q_limits();
    static std::tuple<const VectorXd, const VectorXd> _get_q_dot_limits();
    static DQ_SerialManipulatorMDH kinematics();
};

}
#endif

