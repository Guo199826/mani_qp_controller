#include "../include/tmprod.h"
// Remember to set Rowmajor tensor! TensorMap<Tensor<double,3>, RowMajor>
// This funciton only for the tensors whose rank is 3
// // Function to calculate the n-mode product of a tensor and a matrix
Tensor<double, 3> tmprod(const Eigen::Tensor<double, 3>& T, const Eigen::MatrixXd& M, const int mode){
    Tensor<double, 3> result;
    Tensor<double, 3> T_temp;
    array<long, 3> size_tens;
    int rank = 3; //=3
    for (int i = 0; i<rank; i++){
        int temp = T.dimension(i);
        size_tens[i] = temp;
    }
    array<int, 3> perm;
    switch (mode) {
    case 1:
        perm = {0,1,2};
        break;

    case 2:
        perm = {1,0,2};
        break;

    case 3:
        perm = {2,0,1};
        break;
    }
    size_tens={size_tens[perm[0]], size_tens[perm[1]], size_tens[perm[2]]};
    T_temp = T;
    if (mode != 1){
        result = T_temp.shuffle(perm);}
    else{
            result = T_temp;
    }
    // std::cout<<"result------------------------- "<<std::endl<<result<<std::endl;
    // nmode product:
    size_tens[0] = M.rows();
    int col = result.size()/result.dimension(0);
    MatrixXd M_temp = Map<MatrixXd> (result.data(), result.dimension(0),col);
    MatrixXd M_temp2 = M* M_temp;
    // std::cout<<"M_temp2--------------------------- "<<std::endl<<M_temp2<<std::endl;
    Tensor<double,3> T_temp2 = TensorMap<Tensor<double,3>>(M_temp2.data(), size_tens[0],size_tens[1],size_tens[2]);
    // Tensor<double,3> T_temp2 = TensorMap<Tensor<double,3>>(M_temp2.data(), size_tens[0],size_tens[1],size_tens[2]);
    array<int, 3> iperm;
    iperm[perm[0]]=0;
    iperm[perm[1]]=1;
    iperm[perm[2]]=2;
    Tensor<double,3> T_temp3 = T_temp2.shuffle(iperm);

    return T_temp3;
}


    
