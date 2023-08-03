#ifndef SPDTOVEC_H
#define SPDTOVEC_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>

using namespace Eigen;

VectorXd spd2vec_vec (const MatrixXd &M);
MatrixXd spd2vec_mat (const Tensor<double,3> &T);

#endif