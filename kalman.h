#pragma once
#include <Eigen/Dense>
#include "exception.h"

namespace observer {
// TODO: Implement linear system interface instead of assuming Bicycle
// TODO: Enforce that linear system discrete time matrices are computed

template<typename T>
class Kalman {
    public:
//        using n = typename T::n; // system state size
//        using m = typename T::m; // system input size
//        using l = typename T::l; // system output size

        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;
        using kalman_gain_t = typename Eigen::Matrix<double, T::n, T::l>;
        using error_covariance_t = typename T::state_matrix_t;
        using process_noise_covariance_t = typename T::state_matrix_t;
        using measurement_noise_covariance_t = typename Eigen::Matrix<double, T::l, T::l>;

        Kalman(T& system, const process_noise_covariance_t& Q,
                const measurement_noise_covariance_t& R, const state_t& x0,
                const error_covariance_t& P0) :
            m_system(system), m_x(x0), m_P(P0), m_Q(Q), m_R(R) { }

        void time_update();
        void time_update(const process_noise_covariance_t& Q);
        void time_update(const input_t& u);
        void time_update(const input_t& u, const process_noise_covariance_t& Q);
        void measurement_update(const measurement_t& z);
        void measurement_update(const measurement_t& z, const measurement_noise_covariance_t& R);

        // accessors
        T& system() const;
        state_t x() const;
        kalman_gain_t K() const;
        error_covariance_t P() const;
        process_noise_covariance_t Q() const;
        measurement_noise_covariance_t R() const;
        double dt() const;

    private:
        T& m_system;
        state_t m_x;
        kalman_gain_t m_K;
        error_covariance_t m_P;
        process_noise_covariance_t m_Q;
        measurement_noise_covariance_t m_R;

        void time_update_state();
        void time_update_state(const input_t& u);
        void time_update_error_covariance();
        void time_update_error_covariance(const process_noise_covariance_t& Q);
        void measurement_update_kalman_gain();
        void measurement_update_kalman_gain(const measurement_noise_covariance_t& R);
        void measurement_update_state(const measurement_t& z);
        void measurement_update_error_covariance();
}; // class Kalman


template<typename T>
inline T& Kalman<T>::system() const {
    return m_system;
}

template<typename T>
inline typename Kalman<T>::state_t Kalman<T>::x() const {
    return m_x;
}

template<typename T>
inline typename Kalman<T>::kalman_gain_t Kalman<T>::K() const {
    return m_K;
}

template<typename T>
inline typename Kalman<T>::error_covariance_t Kalman<T>::P() const {
    return m_P;
}

template<typename T>
inline typename Kalman<T>::process_noise_covariance_t Kalman<T>::Q() const {
    return m_Q;
}

template<typename T>
inline typename Kalman<T>::measurement_noise_covariance_t Kalman<T>::R() const {
    return m_R;
}

template<typename T>
inline double Kalman<T>::dt() const {
    return m_system.dt();
}

} // namespace observer

#include "kalman.hh"
