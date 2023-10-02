#include "../include/save_data.h"

void saveData(std::string fileName, Eigten::MatrixXd  matrix)
{
	//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
	const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

	ofstream file(fileName);
	if (file.is_open())
	{
		file << matrix.format(CSVFormat);
		file.close();
	}
}