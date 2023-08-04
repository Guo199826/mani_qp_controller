#include "../include/symm2vec.h"

VectorXd symm2vec (const MatrixXd &mat){
    VectorXd res(mat.rows()*(mat.cols()+1)/2);
    Index size = mat.rows();
    Index offset = 0;
    for(Index j=0; j<mat.cols(); ++j) {
        res.segment(offset,size) = mat.col(j).tail(size);
        offset += size;
        size--;
    }   
    return res;
}
