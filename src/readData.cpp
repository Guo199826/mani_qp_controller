#include "../include/readData.h"

void saveDataMatrix(std::string fileName, MatrixXd matrix)
{
    /// Output and save the Eigen::Tensor data to a file
    std::ofstream outFile(fileName, std::ios::out | std::ios::binary);
    if (outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(matrix.data()), matrix.size() * sizeof(double));
        outFile.close();
        std::cout << "Matrix data has been saved ..." << std::endl;
    } else {
        std::cerr << "Error: Unable to open the output file." << std::endl;
    }
}
void saveDataTensor(std::string fileName, Tensor<double,3> tensor)
{
    /// Output and save the Eigen::Tensor data to a file
    std::ofstream outFile(fileName, std::ios::out | std::ios::binary);
    if (outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(tensor.data()), tensor.size() * sizeof(double));
        outFile.close();
        std::cout << "Tensor data has been saved ..." << std::endl;
    } else {
        std::cerr << "Error: Unable to open the output file." << std::endl;
    }
}

MatrixXd readDataMatrix(std::string datapath, int dimension1, int dimension2){
    std::ifstream inFile(datapath, std::ios::in | std::ios::binary);
    MatrixXd loadedMatrix;
    if (inFile.is_open()) {
        // Get the file size to create a buffer of appropriate size
        inFile.seekg(0, std::ios::end);
        std::streampos fileSize = inFile.tellg();
        inFile.seekg(0, std::ios::beg);

        // Allocate a buffer to hold the data
        std::vector<double> buffer(fileSize / sizeof(double));
        inFile.read(reinterpret_cast<char*>(buffer.data()), fileSize);

        // Close the file after reading
        inFile.close();

        // Create an Eigen::Tensor using the loaded data
        // Eigen::TensorMap<Eigen::Tensor<double, 3>> loadedTensor(buffer.data(), 6, 6, 5);
        loadedMatrix = Map<MatrixXd> (buffer.data(), dimension1, dimension2);

        // Print the loaded tensor
        std::cout << "Loaded Matrix:" << std::endl;
        std::cout << loadedMatrix << std::endl;
    } else {
        std::cerr << "Error: Unable to open the input file." << std::endl;
    }
    return loadedMatrix;
}

Tensor<double,3> readDataTensor(std::string datapath, int dimension1, int dimension2, int dimension3){
    std::ifstream inFile(datapath, std::ios::in | std::ios::binary);
    Tensor<double, 3>loadedTensor;
    if (inFile.is_open()) {
        // Get the file size to create a buffer of appropriate size
        inFile.seekg(0, std::ios::end);
        std::streampos fileSize = inFile.tellg();
        inFile.seekg(0, std::ios::beg);

        // Allocate a buffer to hold the data
        std::vector<double> buffer(fileSize / sizeof(double));
        inFile.read(reinterpret_cast<char*>(buffer.data()), fileSize);

        // Close the file after reading
        inFile.close();
        // Create an Eigen::Tensor using the loaded data
        // Eigen::TensorMap<Eigen::Tensor<double, 3>> loadedTensor(buffer.data(), 6, 6, 5);
        loadedTensor = TensorMap<Tensor<double,3>> (buffer.data(), dimension1, dimension2, dimension3);

        // Print the loaded tensor
        std::cout << "Loaded Tensor:" << std::endl;
        std::cout << loadedTensor << std::endl;
        // std::cout<< "T(3,4,2): "<<loadedTensor(2,3)<<std::endl;
    } else {
        std::cerr << "Error: Unable to open the input file." << std::endl;
    }
    return loadedTensor;
}