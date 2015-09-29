#pragma once
#include <type_traits>
#include <Eigen/Core>
#include "discrete_linear.h"

namespace controller {
// TODO: Allow reference state to vary in horizon

template<typename T>
class Lqr {
    static_assert(std::is_base_of<model::DiscreteLinearBase, T>::value, "Invalid template parameter type for Lqr");
    public:
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using lqr_gain_t = typename Eigen::Matrix<double, T::m, T::n>;
        using state_cost_t = typename T::state_matrix_t;
        using input_cost_t = typename Eigen::Matrix<double, T::m, T::m>;

        Lqr(T& system, const state_cost_t& Q, const input_cost_t& R,
                const state_t& r, uint32_t horizon_iterations) :
            m_system(system), m_horizon(horizon_iterations),
            m_r(r), m_Q(Q), m_R(R),
            m_steady_state(false), m_Ad(system.Ad()), m_Bd(system.Bd()) { }

        input_t control_calculate(const state_t& x);

        void set_horizon(uint32_t horizon_iterations);
        void set_reference(const state_t& r);
        void set_Q(const state_cost_t& Q);
        void set_R(const input_cost_t& R);

        // accessors
        T& system() const;
        uint32_t horizon_iterations() const;
        state_t r() const;
        lqr_gain_t K() const;
        state_cost_t P() const;
        state_cost_t Q() const;
        input_cost_t R() const;
        state_t x() const;
        input_t u() const;
        double dt() const;

    private:
        T& m_system;
        uint32_t m_horizon;
        state_t m_x; // starting state
        state_t m_r; // reference state
        input_t m_u; // starting state
        lqr_gain_t m_K;
        state_cost_t m_P; // running horizon cost
        state_cost_t m_Q;
        input_cost_t m_R;
        bool m_steady_state;
        typename T::state_matrix_t m_Ad;
        typename T::input_matrix_t m_Bd;

        void update_reference();
        void update_lqr_gain();
        void update_horizon_cost();
}; // class Lqr

template<typename T>
inline void Lqr<T>::set_horizon(uint32_t horizon_iterations) {
    m_steady_state = false;
    m_horizon = horizon_iterations;
}

template<typename T>
inline void Lqr<T>::set_reference(const state_t& r) {
    m_steady_state = false;
    m_r = r;
}

template<typename T>
inline void Lqr<T>::set_Q(const state_cost_t& Q) {
    m_steady_state = false;
    m_Q = Q;
}

template<typename T>
inline void Lqr<T>::set_R(const input_cost_t& R) {
    m_steady_state = false;
    m_R = R;
}

template<typename T>
inline T& Lqr<T>::system() const {
    return m_system;
}

template<typename T>
inline uint32_t Lqr<T>::horizon_iterations() const {
    return m_horizon;
}

template<typename T>
inline typename Lqr<T>::state_t Lqr<T>::r() const {
    return m_r;
}

template<typename T>
inline typename Lqr<T>::lqr_gain_t Lqr<T>::K() const {
    return m_K;
}

template<typename T>
inline typename Lqr<T>::state_cost_t Lqr<T>::P() const {
    return m_P;
}

template<typename T>
inline typename Lqr<T>::state_cost_t Lqr<T>::Q() const {
    return m_Q;
}

template<typename T>
inline typename Lqr<T>::input_cost_t Lqr<T>::R() const {
    return m_R;
}

template<typename T>
inline typename Lqr<T>::state_t Lqr<T>::x() const {
    return m_x;
}

template<typename T>
inline typename Lqr<T>::input_t Lqr<T>::u() const {
    return m_u;
}

template<typename T>
inline double Lqr<T>::dt() const {
    return m_system.dt();
}

} // namespace controller

#include "lqr.hh"
