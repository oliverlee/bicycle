#pragma once
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include "types.h"

namespace test {
using matrix_xr_t = Eigen::Matrix<model::real_t, Eigen::Dynamic, Eigen::Dynamic>;

std::string output_matrices(matrix_xr_t expected, matrix_xr_t actual) {
    std::stringstream ss;
    ss << "expected:\n" << expected << "\nactual:\n" << actual << std::endl;
    return ss.str();
}

/*
 * allclose() function to match numpy.allclose
 * https://stackoverflow.com/questions/15051367/how-to-compare-vectors-approximately-in-eigen
 */
template<typename DerivedA, typename DerivedB>
bool allclose(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& reltol
                = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& abstol
                = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon()) {
  return ((a.derived() - b.derived()).array().abs() <= (abstol + reltol * b.derived().array().abs())).all();
}

} // namespace test
