//
// Created by skywoodsz on 2023/3/19.
//

#ifndef SRC_DATAIO_H
#define SRC_DATAIO_H
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <vector>

using namespace std;
using namespace Eigen;

void saveData(string fileName, Eigen::MatrixXd matrix) {
    //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

    ofstream file(fileName);

    file << matrix.format(CSVFormat);
//    file.close();

}

#endif //SRC_DATAIO_H
