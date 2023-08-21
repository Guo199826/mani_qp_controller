#include "../include/q2x.h"

MatrixXd q2x(const MatrixXd &q_traj, const size_t col){
    std::cout<<"I'm in"<<std::endl;
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
    MatrixXd x_t_traj(3, col);
    std::cout<<"x_t_traj init: "<<x_t_traj(1,0)<<std::endl;
    for(size_t i=0; i<col; i++){
        std::cout<<"before vector assign"<<std::endl;
        VectorXd q = q_traj.col(i);
        // forward kinematic model
        DQ xt = robot.fkm(q);
        Matrix<double,3,1> xt_t = xt.translation().vec3();
        std::cout<<"before assign"<<std::endl;
        x_t_traj.col(i) = xt_t;
    }
    std::cout<<"x_t_traj: "<<x_t_traj<<std::endl;
    
    return x_t_traj;
}