#ifndef READDATA_H
#define READDATA_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <fstream>

using namespace Eigen;

void saveDataMatrix(std::string fileName, MatrixXd matrix);
void saveDataTensor(std::string fileName, Tensor<double,3> tensor);
MatrixXd readDataMatrix(std::string datapath, int dimension1, int dimension2);
Tensor<double,3> readDataTensor(std::string datapath, int dimension1, int dimension2, int dimension3);

#endif