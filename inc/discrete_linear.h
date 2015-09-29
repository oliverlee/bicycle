#pragma once
#include <Eigen/Core>

namespace model {

class DiscreteLinearBase {
    public:
        virtual ~DiscreteLinearBase()=0;
};

inline DiscreteLinearBase::~DiscreteLinearBase() { }

template <size_t N, size_t M, size_t L, size_t O>
class DiscreteLinear : private DiscreteLinearBase {
    public:
        static constexpr unsigned int n = N; // state size
        static constexpr unsigned int m = M; // input size
        static constexpr unsigned int l = L; // output size
        static constexpr unsigned int o = O; // second order state size

        using state_t = Eigen::Matrix<double, n, 1>;
        using input_t = Eigen::Matrix<double, m, 1>;
        using output_t = Eigen::Matrix<double, l, 1>;
        using state_matrix_t = Eigen::Matrix<double, n, n>;
        using input_matrix_t = Eigen::Matrix<double, n, m>;
        using output_matrix_t = Eigen::Matrix<double, l, n>;
        using feedthrough_matrix_t = Eigen::Matrix<double, l, m>;
        using second_order_matrix_t = Eigen::Matrix<double, o, o>;

        virtual state_t x_next(const state_t& x, const input_t& u) const = 0;
        virtual output_t y(const state_t& x, const input_t& u) const = 0;
        virtual state_t x_next(const state_t& x) const = 0;
        virtual output_t y(const state_t& x) const = 0;
        virtual state_matrix_t Ad() const = 0;
        virtual input_matrix_t Bd() const = 0;
        virtual output_matrix_t Cd() const = 0;
        virtual feedthrough_matrix_t Dd() const = 0;
        virtual double dt() const = 0;
};

} // namespace model
