#include "../include/manipulabilityJacobian.h"

Tensor<double, 3> manipulabilityJacobian(const MatrixXd &geomJ, const Tensor<double, 3> &J_grad){
    array<int, 3> perm{1,0,2};
    Tensor<double, 3> T_temp = J_grad.shuffle(perm);
    Tensor<double, 3> Jm = tmprod(J_grad, geomJ,2) + tmprod(T_temp, geomJ, 1);
    // int d = Jm.dimension(1);
    // int d_2 = Jm.dimension(2);
    // std::cout<<d<<std::endl;
    // std::cout<<d_2<<std::endl;

    return Jm;
}
