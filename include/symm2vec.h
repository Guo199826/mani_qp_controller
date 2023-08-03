#ifndef SYMM2VEC_H
#define SYMM2VEC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <functional>

using namespace Eigen;

VectorXd symm2vec (const MatrixXd &mat);

#endif