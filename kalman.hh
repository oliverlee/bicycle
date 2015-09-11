/*
 * Member function definitions of Kalman template class
 * See kalman.h for template class declaration.
 */

namespace observer {

template<typename T>
void Kalman<T>::time_update() {
    time_update_state();
    time_update_error_covariance();
}

template<typename T>
void Kalman<T>::time_update(const process_noise_covariance_t& Q) {
    time_update_state();
    time_update_error_covariance(Q);
}

template<typename T>
void Kalman<T>::time_update(const input_t& u) {
    time_update_state(u);
    time_update_error_covariance();
}

template<typename T>
void Kalman<T>::time_update(const input_t& u, const process_noise_covariance_t& Q) {
    time_update_state(u);
    time_update_error_covariance(Q);
}

template<typename T>
void Kalman<T>::measurement_update(const measurement_t& z) {
    measurement_update_kalman_gain();
    measurement_update_state(z);
    measurement_update_error_covariance();
}

template<typename T>
void Kalman<T>::measurement_update(const measurement_t& z, const measurement_noise_covariance_t& R) {
    measurement_update_kalman_gain(R);
    measurement_update_state(z);
    measurement_update_error_covariance();
}

template<typename T>
void Kalman<T>::time_update_state() {
    m_x = m_system.Ad()*m_x;
}

template<typename T>
void Kalman<T>::time_update_state(const input_t& u) {
    m_x = m_system.Ad()*m_x + m_system.Bd()*u;
}

template<typename T>
void Kalman<T>::time_update_error_covariance() {
    m_P = m_system.Ad()*m_P*m_system.Ad().transpose() + m_Q;
}

template<typename T>
void Kalman<T>::time_update_error_covariance(const process_noise_covariance_t& Q) {
    m_P = m_system.Ad()*m_P*m_system.Ad().transpose() + Q;
}

template<typename T>
void Kalman<T>::measurement_update_kalman_gain() {
    m_K = m_P*m_system.Cd().transpose()*(m_system.Cd()*m_P*m_system.Cd().transpose() + m_R);
}

template<typename T>
void Kalman<T>::measurement_update_kalman_gain(const measurement_noise_covariance_t& R) {
    m_K = m_P*m_system.Cd().transpose()*(m_system.Cd()*m_P*m_system.Cd().transpose() + R);
}

template<typename T>
void Kalman<T>::measurement_update_state(const measurement_t& z) {
    m_x = m_x + m_K*(z - m_system.Cd()*m_x);
}

template<typename T>
void Kalman<T>::measurement_update_error_covariance() {
    m_P = (error_covariance_t::Identity() - m_K*m_system.Cd())*m_P;
}

} // namespace observer
