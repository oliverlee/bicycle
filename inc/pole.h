#pragma once
#include <complex>
#include <Eigen/Core>

namespace pole {

using real_t = model::real_t;
using pole_t = std::complex<real_t>;

namespace internal {

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<real_t, DerivedB::ColsAtCompileTime, DerivedB::RowsAtCompileTime>
    place_full_rank(const Eigen::MatrixBase<DerivedA>& A,
                    const DerivedB& qr_B,
                    const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedA::RowsAtCompileTime, 1>>& poles);

template <typename DerivedS, typename DerivedX, typename DerivedB>
void yt_loop(Eigen::MatrixBase<DerivedS>& S,
             Eigen::MatrixBase<DerivedX>& X,
             const Eigen::MatrixBase<DerivedB>& B,
             const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedB::RowsAtCompileTime, 1>>& poles);

} // namespace internal


pole_t continuous_to_discrete(const pole_t& p, real_t dt) { return std::exp(p*dt); }

template <typename T>
T continuous_to_discrete(const Eigen::DenseBase<T>& poles, real_t dt);

template <typename T>
T sort(const Eigen::DenseBase<T>& poles);

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<real_t, DerivedB::ColsAtCompileTime, DerivedB::RowsAtCompileTime>
    place(const Eigen::MatrixBase<DerivedA>& A,
          const Eigen::MatrixBase<DerivedB>& B,
          const Eigen::MatrixBase<Eigen::Matrix<pole_t, DerivedA::RowsAtCompileTime, 1>>& poles);

} // namespace pole

#include "pole.hh"
