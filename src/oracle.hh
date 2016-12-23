/*
 * Member function definitions of Oracle template class.
 * See oracle.h for template class declaration.
 */

namespace observer {

template <typename T>
Oracle<T>::Oracle(T& system) : m_system(system) {
    reset();
}

template <typename T>
Oracle<T>::Oracle(T& system, const state_t& x0) :
    m_system(system), m_state(x0) { }

template <typename T>
void Oracle<T>::reset() {
    m_state.setZero();
}

template <typename T>
void Oracle<T>::update_state(const input_t& u, const measurement_t& z) {
    m_state = m_system.update_state(m_state, u, z);
}

} // namespace observer
