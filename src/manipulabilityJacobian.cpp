#include "../include/manipulabilityJacobian.h"

Tensor<double, 3> manipulabilityJacobian(const MatrixXd &geomJ, const Tensor<double, 3> &J_grad){
    array<int, 3> perm{1,0,2};
    Tensor<double, 3> T_temp = J_grad.shuffle(perm);
    Tensor<double, 3> Jm = tmprod(J_grad, geomJ, 2) + tmprod(T_temp, geomJ, 1);
    // int d = Jm.dimension(1);
    // int d_2 = Jm.dimension(2);
    // std::cout<<d<<std::endl;
    // std::cout<<d_2<<std::endl;

    return Jm;
}


Tensor<double, 3> manipulabilityJacobianDyn(const MatrixXd &L_ct, const Tensor<double, 3> &J_grad,
                    const Tensor<double, 3> &mass_diff_, const Matrix<double, 7, 7> &Mass)
{
    MatrixXd Mass_temp = (Mass.inverse()).transpose();
    array<int, 3> perm{1,0,2};
    Tensor<double, 3> tmprod_temp = tmprod(mass_diff_, L_ct, 1);
    Tensor<double, 3> J_L = tmprod(J_grad, Mass_temp, 2) - tmprod(tmprod_temp, Mass_temp, 2);
    Tensor<double, 3> T_temp = J_L.shuffle(perm);
    Tensor<double, 3> Jm = tmprod(J_L, L_ct, 2) + tmprod(T_temp, L_ct, 1);
    // int d = Jm.dimension(1);
    // int d_2 = Jm.dimension(2);
    // std::cout<<d<<std::endl;
    // std::cout<<d_2<<std::endl;

    return Jm;
}