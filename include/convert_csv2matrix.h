#ifndef CONVERT_CSV2MATRIX_H
#define CONVERT_CSV2MATRIX_H
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>


Eigen::MatrixXd load_csv (const std::string & path);
Eigen::MatrixXd load_csv_8 (const std::string & path);
Eigen::MatrixXd load_csv64 (const std::string & path);

// Eigen::MatrixXd load_txt (const std::string & path);

#endif