#ifndef DQ_ROBOTS_FRANKAROBOT_H
#define DQ_ROBOTS_FRANKAROBOT_H

#include<dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

class FrankaRobot
{
public:
    static MatrixXd _get_mdh_matrix();
    static DQ _get_offset_base();
    static DQ _get_offset_flange();
    static std::tuple<const VectorXd, const VectorXd> _get_q_limits();
    static std::tuple<const VectorXd, const VectorXd> _get_q_dot_limits();
    static DQ_SerialManipulatorMDH kinematics();
    // {
    //     const double pi2 = pi/2.0;

    //     MatrixXd franka_dh(5,7);
      
    //     franka_dh <<    0,      0,          0,          0,      0,      0,          0,
    //                 0.333,      0,      0.316,          0,  0.384,      0,      0.107,
    //                     0,      0,     0.0825,    -0.0825,      0,  0.088,     0.0003,
    //                  -pi2,    pi2,        pi2,       -pi2,    pi2,    pi2,          0,
    //                     0,      0,          0,          0,      0,      0,          0;

    //     DQ_SerialManipulatorDH franka(franka_dh);

    //     return franka;
    // }
};

}

#endif
