#pragma once
#include <complex>
#include <type_traits>
#include <unsupported/Eigen/MatrixFunctions>

namespace pole {

using pole_t = std::complex<model::real_t>;

pole_t continuous_to_discrete(const pole_t& p, model::real_t dt) {
    return std::exp(p*dt);
}

template <typename T>
T continuous_to_discrete(const Eigen::DenseBase<T>& poles, model::real_t dt) {
    static_assert(std::is_same<typename T::Scalar, pole_t>::value,
            "Invalid scalar type converting poles from continuous to discrete time");
    static_assert(
            (T::ColsAtCompileTime == 1) || (T::RowsAtCompileTime == 1),
            "Input must be a vector of poles");

    T discrete_poles;
    for (unsigned int i = 0; i < poles.size(); ++i) {
        discrete_poles(i) = continuous_to_discrete(poles(i), dt);
    }
    return discrete_poles;
}

} // namespace pole
