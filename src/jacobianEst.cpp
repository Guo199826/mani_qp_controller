#include "../include/jacobianEst.h"

Tensor<double, 3> jacobianEst(std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac, const VectorXd& q, const int n,
    const DQ_SerialManipulator &robot) 
{
    double q_delta = 0.0001;
    VectorXd q_add(n);
    q_add.setZero();
    
    VectorXd q_ii;
    VectorXd q_i;
    // Matrix vs MatrixXd??? Matrix could not be returned??
    Matrix<double, 8, 7> J_ii;
    Matrix<double, 8, 7> J_i;
    Matrix<double, 6, 7> J_geom_ii;
    Matrix<double, 6, 7> J_geom_i;

    Tensor<double, 3> JE(6, 7, 7);

    //test:
    Matrix<double, 8, 7> J;
    J = robot.pose_jacobian(q);
    Matrix<double, 6, 7> J_geom;
    J_geom = fct_geomJac(robot,J,q,n);
    std::cout<<"J_geom at q: "<<std::endl<<J_geom<<std::endl;

    Matrix<double, 6, 7> J_result;

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

        // Calculate the jacobian of geom jacobian
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {6, 7, 1};
        J_result = (J_geom_ii - J_geom_i)/(2*q_delta);
        // std::cout<<"J_result"<<std::endl<<J_result(3,5)<<std::endl;
        // constexpr int rank = sizeof... (Dims);
        Tensor<double, 3> J_result_m2t = TensorMap<Tensor<double, 3>>(J_result.data(), 6, 7, 1);
        // std::cout<<"Check result of (:,:,i): "<<std::endl<<J_result_m2t<<std::endl;
        // Tensor<double, 3> slice = JE.slice(offset, extent);
        JE.slice(offset, extent) = J_result_m2t;
        // std::cout<<"Jacobian of Jacobian: "<<i<<std::endl<<JE<<std::endl;
        q_add(i) = 0.0;
    }
    
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(2,1,4)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(4,4,3)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(1,3,1)<<std::endl;

    return JE;
}
