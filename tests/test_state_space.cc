#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include "gtest/gtest.h"
#include "bicycle.h"

/*
 * A and B expected state space matrices generated using dtk.bicycle:
 *
 * from dtk.bicycle import *
 * np.set_printoptions(precision=16)
 * A, B = benchmark_state_space(*benchmark_matrices(), v=1, g=9.80665)
 */

namespace {
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 2> B(
        (Eigen::Matrix<double, 4, 2>() <<
            0.                ,  0.                ,
            0.                ,  0.                ,
            0.0159349789179135, -0.1240920254115741,
           -0.1240920254115741,  4.3238401808042282).finished()
        );

    std::string output_matrices(Eigen::MatrixXd expected, Eigen::MatrixXd actual) {
        std::stringstream ss;
        ss << "expected:\n" << expected << "\nactual:\n" << actual << std::endl;
        return ss.str();
    }
} // namespace

TEST(StateSpace, ContinuousV1) {
    model::Bicycle bicycle("benchmark_matrices.txt", 1.0);

    A <<  0.                ,   0.                ,   1.                , 0.                ,
          0.                ,   0.                ,   0.                , 1.                ,
          9.4865338000460664,  -1.4625257433243051,  -0.1055224498056882, -0.330515398992312 ,
         11.7154748079957685,  28.9264833312917631,   3.6768052333214327, -3.0848655274330694;

    EXPECT_TRUE(bicycle.A().isApprox(A)) << output_matrices(bicycle.A(), A);
    EXPECT_TRUE(bicycle.B().isApprox(B)) << output_matrices(bicycle.B(), B);
}

TEST(StateSpace, ContinuousV3) {
    model::Bicycle bicycle("benchmark_matrices.txt", 3.0);

    A <<  0.                ,   0.                ,   1.                , 0.                ,
          0.                ,   0.                ,   0.                , 1.                ,
          9.4865338000460664,  -8.5921076477970253,  -0.3165673494170646, -0.9915461969769359,
         11.7154748079957685,  13.1527626512942426,  11.0304156999642977, -9.2545965822992091;

    EXPECT_TRUE(bicycle.A().isApprox(A)) << output_matrices(bicycle.A(), A);
    EXPECT_TRUE(bicycle.B().isApprox(B)) << output_matrices(bicycle.B(), B);
}

TEST(StateSpace, ContinuousV5) {
    model::Bicycle bicycle("benchmark_matrices.txt", 5.0);

    A <<  0.                ,   0.                ,   1.                , 0.                ,
          0.                ,   0.                ,   0.                , 1.                ,
          9.4865338000460664, -22.8512714567424666,  -0.5276122490284411, -1.6525769949615603,
         11.7154748079957685, -18.3946787087007344,  18.384026166607164 , -15.4243276371653479;

    EXPECT_TRUE(bicycle.A().isApprox(A)) << output_matrices(bicycle.A(), A);
    EXPECT_TRUE(bicycle.B().isApprox(B)) << output_matrices(bicycle.B(), B);
}
