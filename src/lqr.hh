/*
 * Member function definitions of Lqr template class.
 * See lqr.h for template class declaration.
 */

namespace controller {

template<typename T>
typename Lqr<T>::input_t Lqr<T>::control_calculate(const state_t& x) {
    m_x = m_xt; // m_x will be updated backwards in time to adjust target
    for (unsigned int i = 0; i < m_horizon; ++i) {
        update_target();
        update_lqr_gain();
        update_horizon_cost();
    }
    m_u.noalias() = m_K*(x - m_x);
    m_x = x;
    return u();
}

template<typename T>
void Lqr<T>::update_target() {
    if (!m_x.isZero()) {
        m_x = m_system.Ad().fullPivHouseholderQr().solve(m_x).eval();
    }
}

template<typename T>
void Lqr<T>::update_lqr_gain() {
    input_cost_t M = m_R  + m_system.Bd().transpose()*m_P*m_system.Bd();
    m_K.noalias() = -M.fullPivHouseholderQr().solve(m_system.Bd().transpose()*m_P*m_system.Ad());
}

template<typename T>
void Lqr<T>::update_horizon_cost() {
    state_cost_t M = m_system.Ad() + m_system.Bd()*m_K;
    m_P = m_Q + m_K.transpose()*m_R*m_K + M.transpose()*m_P*M;
}

} // namespace controller
