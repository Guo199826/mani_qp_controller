#include "../include/convert_csv2matrix.h"

Eigen::MatrixXd load_csv(const std::string & path) {
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
    // Eigen::MatrixXd q_traj_csv = matrix.block(0,0,7,columns[0].size());
    Eigen::MatrixXd q_traj_csv = matrix.block(0,0,7,columns[0].size());
    
    return q_traj_csv;
}






Eigen::MatrixXd load_csv_3 (const std::string & path) {
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
    // Create an Eigen::MatrixXd from the columns
    Eigen::MatrixXd matrix(columns.size(), columns[0].size());
    for (size_t col = 0; col < columns.size(); ++col) {
        for (size_t row = 0; row < columns[col].size(); ++row) {
            matrix(col, row) = columns[col][row];
        }
    }
    Eigen::MatrixXd q_traj_csv = matrix.block(0,0,3,columns[0].size());

    
    return q_traj_csv;
}

// Eigen::MatrixXd load_txt (const std::string & path) {
//     std::ifstream file(path);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open the file: " << path << std::endl;
//     }
//     std::vector<double> data;
//     std::string line;
//     Eigen::MatrixXd matrix;

//     // Read the file line by line and parse it
//     while (std::getline(file, line,' ')) {
//         std::cout<<"Line:"<<line<<"end"<<std::endl;
//         if (line==" ") {
//             // Empty line indicates a separator between vectors
//             if (!data.empty()) {
//                 // Create an Eigen::Map to the data and add it to the matrix
//                 Eigen::Map<Eigen::VectorXd> vector_data(data.data(), data.size());
//                 if (matrix.size() == 0) {
//                     matrix = vector_data;
//                 } else {
//                     matrix.conservativeResize(matrix.rows(), matrix.cols() + 1);
//                     matrix.col(matrix.cols() - 1) = vector_data;
//                 }
//                 data.clear();
//             }
//         } else {
//             std::cout<<"I'm here!"<<std::endl;
//             // Parse the line as a double and add it to the data vector
//             double value = std::stod(line);
//             if(!std::isnan(value)) {
//                 std::cout<<"value: "<<value<<std::endl;
//                 data.push_back(value);
//             }
            
//         }
//     }

//     // Close the file
//     file.close();

//     // Check if there is any remaining data after the loop
//     if (!data.empty()) {
//         Eigen::Map<Eigen::VectorXd> vector_data(data.data(), data.size());
//         if (matrix.size() == 0) {
//             matrix = vector_data;
//         } else {
//             matrix.conservativeResize(matrix.rows(), matrix.cols() + 1);
//             matrix.col(matrix.cols() - 1) = vector_data;
//         }
//     }

//     // Print the loaded matrix
//     std::cout << "Loaded matrix:" << std::endl;
//     std::cout << matrix.rows()<<"   "<<matrix.cols() << std::endl;

//     return matrix;
// }


    
    