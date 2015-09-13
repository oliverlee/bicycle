#pragma once
#include <string>
#include <sstream>
#include <Eigen/Dense>

std::string output_matrices(Eigen::MatrixXd expected, Eigen::MatrixXd actual) {
    std::stringstream ss;
    ss << "expected:\n" << expected << "\nactual:\n" << actual << std::endl;
    return ss.str();
}
