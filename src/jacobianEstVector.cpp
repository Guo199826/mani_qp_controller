#include "../include/jacobianEstVector.h"
// For derivative of singular value of Jacobian (eigenvalue of M)

MatrixXd jacobianEstVector(std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac, const VectorXd& q, const int n,
    const DQ_SerialManipulator &robot
    ) 
{
    double q_delta = 0.00001;
    VectorXd q_add(n);
    q_add.setZero();
    
    VectorXd q_ii;
    VectorXd q_i;
    // Matrix vs MatrixXd??? Matrix could not be returned??
    Matrix<double, 8, 7> J_ii;
    Matrix<double, 8, 7> J_i;
    Matrix<double, 6, 7> J_geom_ii;
    Matrix<double, 6, 7> J_geom_i;

    BDCSVD<MatrixXd> singularsolver;
    Matrix<double, 6, 1> eigenvalue_i;
    Matrix<double, 6, 1> eigenvalue_ii;
    Matrix<double, 6, 7> JEV;

    //test:
    Matrix<double, 8, 7> J;
    J = robot.pose_jacobian(q);
    Matrix<double, 6, 7> J_geom;
    J_geom = fct_geomJac(robot,J,q,n);
    Matrix<double, 6, 1> eigenvalue;
    eigenvalue = singularsolver.compute(J_geom).singularValues();
    // std::cout<<"Singular value of J_geom at q: "<<std::endl<<eigenvalue<<std::endl;

    for (int i=0; i<n; i++){
        q_add(i) = q_delta;
        q_ii = q + q_add;
        q_i = q - q_add;
        
        J_ii = robot.pose_jacobian(q_ii);
        J_i = robot.pose_jacobian(q_i);
        // std::cout<<"J_ii of "<<i<<std::endl<<J_ii<<std::endl;
        // std::cout<<"J_i: "<<i<<std::endl<<J_i<<std::endl;
        J_geom_ii = fct_geomJac(robot,J_ii,q_ii,n);
        J_geom_i = fct_geomJac(robot,J_i,q_i,n);
        // std::cout<<"J_geom_ii: "<<i<<std::endl<<J_geom_ii<<std::endl;
        // std::cout<<"J_geom_i: "<<i<<std::endl<<J_geom_i<<std::endl;
        eigenvalue_ii = singularsolver.compute(J_geom_ii).singularValues();
        eigenvalue_i = singularsolver.compute(J_geom_i).singularValues();

        JEV.col(i) = (eigenvalue_ii - eigenvalue_i)/(2*q_delta);
        q_add(i) = 0.0;
    }
    return JEV;
    
}
    
