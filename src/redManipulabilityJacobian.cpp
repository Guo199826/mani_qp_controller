#include "../include/manipulabilityJacobian.h"

MatrixXd redManipulabilityJacobian(const MatrixXd &geomJ_, const Tensor<double, 3> &J_grad_){
    Tensor<double, 3> Jm = manipulabilityJacobian(geomJ_, J_grad_);
    MatrixXd Jm_red(21,7);
    Matrix<double,6,6> M_temp;
    Tensor<double, 3> J_result_m2t;
    int size = Jm.dimension(2); // should be 7
    for(int i=0; i<size; i++){
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {6, 6, 1};
        J_result_m2t = Jm.slice(offset, extent);
        M_temp = Map<Matrix<double,6,6>> (J_result_m2t.data(), 6, 6);
        VectorXd Jm_red_i = spd2vec_vec(M_temp);
        Jm_red.col(i)= Jm_red_i;
    }
    // std::cout<<"redManiJacobian: "<<std::endl<<Jm_red<<std::endl;
    return Jm_red;
}

MatrixXd redManipulabilityJacobian_3(const MatrixXd &geomJ_, const Tensor<double, 3> &J_grad_){
    Tensor<double, 3> Jm = manipulabilityJacobian(geomJ_, J_grad_);
    MatrixXd Jm_red(6,7);
    Matrix<double,3,3> M_temp;
    Tensor<double, 3> J_result_m2t;
    int size = Jm.dimension(2); // should be 7
    for(int i=0; i<size; i++){
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {3, 3, 1};
        J_result_m2t = Jm.slice(offset, extent);
        M_temp = Map<Matrix<double,3,3>> (J_result_m2t.data(), 3, 3);
        VectorXd Jm_red_i = spd2vec_vec(M_temp);
        Jm_red.col(i)= Jm_red_i;
    }
    // std::cout<<"redManiJacobian_3: "<<std::endl<<Jm_red<<std::endl;
    return Jm_red;
}