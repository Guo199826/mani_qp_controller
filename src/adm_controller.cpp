// ************************************** Contributor: Yuhe Gong ********************************************
// Admittance Controller Codes
#include "../include/adm_controller.h"


// ***************************** external force control (translation jacobian) ******************************
VectorXd tran_adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext){
    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();

    // Set link number and joint angle
    int n = 7;

    // Calculate Jacobian matrix
    MatrixXd J;
    MatrixXd J_geom;
    Matrix<double,7,1> dq_res;
    J = robot.pose_jacobian(q_);
    J_geom = geomJac(robot, J, q_, n);

    // Cartesian controller law
    double F_ext_threshold = 8;
    Matrix<double, 6, 1> dx;
    dx.setZero();
    if ((F_ext.col(0)).norm() > F_ext_threshold){
        MatrixXd F_ext_normalized = ((F_ext.col(0)).norm() - F_ext_threshold) / (F_ext.col(0)).norm() * F_ext;
        // translation controller
        dx.block(3,0,3,1) = - 0.03 * F_ext_normalized.block(0,0,3,1);
    }

    // Joint velocity signal 
    // CompleteOrthogonalDecomposition<MatrixXd> cod(J_geom);
    // MatrixXd J_geom_use_inv = cod.pseudoInverse();
    // dq_res = J_geom_use_inv * dx.col(0);
    dq_res = J_geom.transpose() * dx.col(0);
    return dq_res;
}


// ***************************** external torque control (rotation jacobian) ******************************
VectorXd rot_adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext){
    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();

    // Set link number and joint angle
    int n = 7;

    // Calculate Jacobian matrix
    MatrixXd J;
    MatrixXd J_geom;
    Matrix<double,7,1> dq_res;
    J = robot.pose_jacobian(q_);
    J_geom = geomJac(robot, J, q_, n);

    // Cartesian controller law
    double F_ext_threshold = 8;
    Matrix<double, 6, 1> dx;
    dx.setZero();
    if ((F_ext.col(0)).norm() > F_ext_threshold){
        MatrixXd F_ext_normalized = ((F_ext.col(0)).norm() - F_ext_threshold) / (F_ext.col(0)).norm() * F_ext;
        // rotation controller:
        dx.block(0,0,3,1) = - 0.15 * F_ext_normalized.block(3,0,3,1);
    }

    // Joint velocity signal 
    // CompleteOrthogonalDecomposition<MatrixXd> cod(J_geom);
    // MatrixXd J_geom_use_inv = cod.pseudoInverse();
    // dq_res = J_geom_use_inv * dx.col(0);
    dq_res = J_geom.transpose() * dx.col(0);
    return dq_res;
}


// *************************** external force & torque control (full jacobian) ****************************
VectorXd full_adm_controller(const Matrix<double,7,1> &q_, const MatrixXd &F_ext){
    
    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();

    // Set link number and joint angle
    int n = 7;

    // Calculate Jacobian matrix
    MatrixXd J;
    MatrixXd J_geom;
    Matrix<double,7,1> dq_res;
    J = robot.pose_jacobian(q_);
    J_geom = geomJac(robot, J, q_, n);

    // Cartesian controller law
    double F_ext_threshold = 5;
    Matrix<double, 6, 1> dx;
    dx.setZero();
    if ((F_ext.col(0)).norm() > F_ext_threshold){
        MatrixXd F_ext_normalized = ((F_ext.col(0)).norm() - F_ext_threshold) / (F_ext.col(0)).norm() * F_ext;
        // translation controller
        dx.block(3,0,3,1) = - 0.08 * F_ext_normalized.block(0,0,3,1);
        // rotation controller:
        dx.block(0,0,3,1) = - 0.3 * F_ext_normalized.block(3,0,3,1);
    }

    // Joint velocity signal 
    CompleteOrthogonalDecomposition<MatrixXd> cod(J_geom);
    MatrixXd J_geom_use_inv = cod.pseudoInverse();
    dq_res = J_geom_use_inv * dx.col(0);
    //dq_res = J_geom.transpose() * dx.col(0);
    return dq_res;
}