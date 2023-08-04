#include "../include/spdToVec.h"

// using Mandel notation from Noemie's code
VectorXd spd2vec_vec (const MatrixXd &M){
    Index n_1 = M.rows();
    VectorXd vec;
    VectorXd vec_add;
    vec = M.diagonal();
    
    for (Index i =1; i<n_1; i++){
        vec_add = sqrt(2)*M.diagonal(i);
        int to_append = vec_add.size();
        vec.conservativeResize(vec.size() + vec_add.size());
        vec.bottomRows(to_append)= vec_add;
        // vec.bottomRows(to_append) = vec_add;
        // vec << vec, vec_add;
    }
    // std::cout<<"spd2vec_vec_test: "<<std::endl<<vec<<std::endl;
    return vec;
}

MatrixXd spd2vec_mat (const Tensor<double,3> &T){
    Tensor<double,3> T_temp;
    int d = T.dimension(0);
    int n = T.dimension(2);
    // std::cout<<"T_dimension 0: "<<d<<std::endl;
    // std::cout<<"T_dimension 2: "<<n<<std::endl;
    VectorXd vec;
    VectorXd vec_add;
    MatrixXd M;
    MatrixXd M_res;
    array<DenseIndex, 3> offset; 
    array<DenseIndex, 3> extent;
    
    for (int i = 0; i<n; i++){
        offset = {0, 0, i};
        extent = {d, d, 1};
        T_temp = T.slice(offset, extent);
        M = Map<MatrixXd> (T_temp.data(), d, d);
        std::cout<<"M: "<<std::endl<<M<<std::endl;
        vec = M.diagonal();
        
        for (int j = 1; j<d; j++){
            vec_add = sqrt(2) * M.diagonal(j);
            int to_append = vec_add.size();
            vec.conservativeResize(vec.size() + vec_add.size(),1);
            vec.bottomRows(to_append)= vec_add;
        }
        M_res.conservativeResize(vec.rows(), i+1);
        // !vec.rows could be calculated forher
        M_res.col(i) = vec;
    }
    // std::cout<<"Result of tensor symm2vec: "<<std::endl<<M_res<<std::endl;
    return M_res;
}