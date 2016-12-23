#pragma once
#include <Eigen/Core>
#include "observer.h"

namespace observer {

/*
 * This template class implements a discrete-time Kalman Filter.
 */
template <typename T>
class Kalman final : public Observer<T> {
    public:
        /*
         * Although state_t, input_t, measurement_t are declared in the Observer<T> class
         * and are inherited, the type aliases are dependent names as the base class (Observer)
         * is a template dependent on the template parameter of Kalman. In order to refer to
         * the type alias, we would need to specify 'typename Kalman::state_t', thus it is
         * easier to simply redeclare the type aliases.
         */
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;
        using kalman_gain_t = typename Eigen::Matrix<real_t, T::n, T::l>;
        using error_covariance_t = typename T::state_matrix_t;
        using process_noise_covariance_t = typename T::state_matrix_t;
        using measurement_noise_covariance_t = typename Eigen::Matrix<real_t, T::l, T::l>;

        Kalman(T& system);
        Kalman(T& system, const state_t& x0);
        Kalman(T& system, const state_t& x0,
                const process_noise_covariance_t& Q,
                const measurement_noise_covariance_t& R,
                const error_covariance_t& P0);

        virtual void reset() override;
        // simplified time and measurement update
        virtual void update_state(const input_t& u, const measurement_t& z) override;

        void time_update();
        void time_update(const process_noise_covariance_t& Q);
        void time_update(const input_t& u);
        void time_update(const input_t& u, const process_noise_covariance_t& Q);
        void measurement_update(const measurement_t& z);
        void measurement_update(const measurement_t& z, const measurement_noise_covariance_t& R);

        virtual void set_state(const state_t& x) override;
        void set_x(const state_t& x);
        void set_P(const error_covariance_t& P);
        void set_Q(const process_noise_covariance_t& Q);
        void set_R(const measurement_noise_covariance_t& R);

        // accessors
        virtual T& system() const override;
        virtual real_t dt() const override;
        virtual const state_t& state() const override;
        const state_t& x() const;
        const kalman_gain_t& K() const;
        const error_covariance_t& P() const;
        const process_noise_covariance_t& Q() const;
        const measurement_noise_covariance_t& R() const;

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

template <typename T>
inline void Kalman<T>::set_state(const state_t& x) {
    set_x(x);
}

template <typename T>
inline void Kalman<T>::set_x(const state_t& x) {
    m_x = x;
}

template <typename T>
inline void Kalman<T>::set_P(const error_covariance_t& P) {
    m_P = P;
}

template <typename T>
inline void Kalman<T>::set_Q(const process_noise_covariance_t& Q) {
    m_Q = Q;
}

template <typename T>
inline void Kalman<T>::set_R(const measurement_noise_covariance_t& R) {
    m_R = R;
}

template <typename T>
inline T& Kalman<T>::system() const {
    return m_system;
}

template <typename T>
inline real_t Kalman<T>::dt() const {
    return m_system.dt();
}

template <typename T>
inline const typename Kalman<T>::state_t& Kalman<T>::state() const {
    return x();
}

template <typename T>
inline const typename Kalman<T>::state_t& Kalman<T>::x() const {
    return m_x;
}

template <typename T>
inline const typename Kalman<T>::kalman_gain_t& Kalman<T>::K() const {
    return m_K;
}

template <typename T>
inline const typename Kalman<T>::error_covariance_t& Kalman<T>::P() const {
    return m_P;
}

template <typename T>
inline const typename Kalman<T>::process_noise_covariance_t& Kalman<T>::Q() const {
    return m_Q;
}

template <typename T>
inline const typename Kalman<T>::measurement_noise_covariance_t& Kalman<T>::R() const {
    return m_R;
}

} // namespace observer

#include "kalman.hh"
