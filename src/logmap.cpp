// Not the same as in Matlab
// input arguments are Matrix instead of Tensor
#include "../include/logmap.h"

MatrixXd logmap(const MatrixXd &M_1, const MatrixXd &M_2){
    int size = M_1.rows();
    PartialPivLU<MatrixXd> lu(M_2); // WHy different from Matlab mldivide????
    MatrixXd div = lu.solve(M_1);
    // std::cout<<"div: "<<std::endl<<div<<std::endl;
    EigenSolver<MatrixXd> eigensolver(div); // WHy different from Matlab eig()????

    auto EV = eigensolver.eigenvalues();
    VectorXcd EV_log = EV.array().log();
    MatrixXcd M_log = EV_log.asDiagonal();
    // std::cout<<"eigenvalue log: "<<std::endl<<EV<<std::endl;
    // std::cout<<"eigenvalue log M: "<<std::endl<<M_log<<std::endl;

    MatrixXcd EVec = eigensolver.eigenvectors();
    // std::cout<<"eigenvector: "<<std::endl<<EVec<<std::endl;
    MatrixXcd M_pow = EVec.inverse();
    MatrixXcd M_temp = M_2 * EVec * M_log * M_pow;
    MatrixXd result(size,size);
    result = M_temp.real();
    // std::cout<<"result: "<<std::endl<<result<<std::endl;
    return result;
}