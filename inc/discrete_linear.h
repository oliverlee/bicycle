#pragma once
#include <Eigen/Core>
#include "types.h"

namespace model {

/* This class cannot be instantiated and does not allow polymorphic deletion through a base pointer.*/
class DiscreteLinearBase {
    protected:
        ~DiscreteLinearBase() { }
};

template <size_t N, size_t M, size_t L, size_t O>
class DiscreteLinear : private DiscreteLinearBase {
    public:
        static constexpr unsigned int n = N; // state size
        static constexpr unsigned int m = M; // input size
        static constexpr unsigned int l = L; // output size
        static constexpr unsigned int o = O; // second order state size

        using state_t = Eigen::Matrix<real_t, n, 1>;
        using input_t = Eigen::Matrix<real_t, m, 1>;
        using output_t = Eigen::Matrix<real_t, l, 1>;
        using state_matrix_t = Eigen::Matrix<real_t, n, n>;
        using input_matrix_t = Eigen::Matrix<real_t, n, m>;
        using output_matrix_t = Eigen::Matrix<real_t, l, n>;
        using feedthrough_matrix_t = Eigen::Matrix<real_t, l, m>;
        using second_order_matrix_t = Eigen::Matrix<real_t, o, o>;

        virtual state_t update_state(const state_t& x, const input_t& u) const = 0;
        virtual output_t calculate_output(const state_t& x, const input_t& u) const = 0;
        virtual state_t update_state(const state_t& x) const = 0;
        virtual output_t calculate_output(const state_t& x) const = 0;

        virtual const state_matrix_t& Ad() const = 0;
        virtual const input_matrix_t& Bd() const = 0;
        virtual const output_matrix_t& Cd() const = 0;
        virtual const feedthrough_matrix_t& Dd() const = 0;
        virtual real_t dt() const = 0;
};

} // namespace model
