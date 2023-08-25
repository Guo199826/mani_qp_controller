#include "../include/adm_controller.h"

VectorXd adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext){
    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
    // Set link number and joint angle
    int n = 7;
    MatrixXd J;
    MatrixXd J_geom;
    MatrixXd J_geom_t;
    Matrix<double,7,1> dq_res;
    J = robot.pose_jacobian(q_);
    J_geom = geomJac(robot, J, q_, n); 
    J_geom_t = J_geom.block(3, 0, 3, n); 

    // ******************* Contributor: Yuhe Gong ******************************
    // Admittance Controller
    // external force threshold
    double F_ext_threshold = 8;
    // control law
    Vector3d control_signal; 
    if ((F_ext.col(0)).norm() > F_ext_threshold ){
        control_signal = - 0.01* F_ext;
    }  
    else {
        control_signal.setZero();
    }
    // eqaulity constraint right part
    Vector3d dxr = control_signal;
    // set the equality constraints: J * \dot{q} = \dot{x}
    CompleteOrthogonalDecomposition<MatrixXd> cod(J_geom_t);
    MatrixXd J_geom_t_inv = cod.pseudoInverse();
    // std::cout<<"pseudoinverse of J_geom_t:\n"<<J_geom_t_inv<<std::endl;
    dq_res = J_geom_t_inv * dxr;
    // **************************************************************************
    return dq_res;
}