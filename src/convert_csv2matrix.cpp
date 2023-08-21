#include "../include/convert_csv2matrix.h"

Eigen::MatrixXd load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);

    std::vector<std::vector<double>> columns; // Store data column-wise
    std::string line;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        size_t col = 0;
        
        while (std::getline(lineStream, cell, ',')) {
            if (cell!=""){
                std::vector<double> column;
                column.push_back(std::stod(cell));
                if (col >= columns.size()) {
                    columns.push_back(std::vector<double>());
                }
                columns[col].push_back(std::stod(cell));
                ++col;
            }
        }
    }
    // std::cout<<"columns size:"<<columns[0].size()<<std::endl;
    // Create an Eigen::MatrixXd from the columns
    Eigen::MatrixXd matrix(columns.size(), columns[0].size());
    for (size_t col = 0; col < columns.size(); ++col) {
        for (size_t row = 0; row < columns[col].size(); ++row) {
            matrix(col, row) = columns[col][row];
        }
    }
    Eigen::MatrixXd q_traj_csv = matrix.block(3,0,7,columns[0].size());

    
    return q_traj_csv;
}

    
    