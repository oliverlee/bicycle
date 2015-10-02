/*
 * Member function definitions of Lqr template class.
 * See lqr.h for template class declaration.
 */

namespace controller {

template<typename T>
Lqr<T>::Lqr(T& system, const state_cost_t& Q, const input_cost_t& R,
        const state_t& r, uint32_t horizon_iterations) :
    m_system(system), m_horizon(horizon_iterations), m_r(r), m_Q(Q), m_R(R),
    m_steady_state(false), m_Ad(system.Ad()), m_Bd(system.Bd()) {
    set_control_mask();
}

template<typename T>
typename Lqr<T>::input_t Lqr<T>::control_calculate(const state_t& x) {
    perform_value_iteration();
    input_t reduced_u = m_K*(x - m_r);
    if (m_m == T::m) {
        //std::cout << "u: " << reduced_u.transpose() << std::endl;
        return reduced_u;
    }

    input_t u = input_t::Zero();
    for (unsigned int i = 0, j = 0; i < T::m; ++i) {
        if (m_mask[i]) {
            u[i] = reduced_u[j++];
        }
    }
    //std::cout << "u: " << u.transpose() << std::endl;
    return u;
}

template<typename T>
typename Lqr<T>::input_t Lqr<T>::control_calculate(const state_t& x, const state_t& r) {
    set_reference(r);
    return control_calculate(x);
}

template<typename T>
void Lqr<T>::perform_value_iteration() {
    if (!m_system.Ad().isApprox(m_Ad) || !m_system.Bd().isApprox(m_Bd)) {
        // check if system has changed
        m_Ad = m_system.Ad();
        m_Bd = m_system.Bd();
        reduce_input_matrices();
        m_steady_state = false;
    }

    if (!m_steady_state) {
        lqr_gain_t K = m_K;
        state_cost_t P = m_P;
        for (unsigned int i = 0; i < m_horizon; ++i) {
            update_lqr_gain();
            update_horizon_cost();
        }
        if (K.isApprox(m_K) && P.isApprox(m_P)) {
            m_steady_state = true;
            //std::cout << "steady state reached" << std::endl;
            //std::cout << "K:\n" << m_K << std::endl;
            //std::cout << "P:\n" << m_P << std::endl;
        }
    }
}

template<typename T>
void Lqr<T>::update_lqr_gain() {
    input_cost_t M;
    if (m_m == T::m) {
        M.noalias() = m_R + m_Bd.transpose()*m_P*m_Bd;
        m_K.noalias() = -M.fullPivHouseholderQr().solve(m_Bd.transpose()*m_P*m_Ad);
    } else {
        M.topLeftCorner(m_m, m_m).noalias() =
            m_Rr.topLeftCorner(m_m, m_m) + m_Br.leftCols(m_m).transpose()*m_P*m_Br.leftCols(m_m);
        m_K.topRows(m_m).noalias() = -M.topLeftCorner(m_m, m_m).fullPivHouseholderQr().solve(
                m_Br.leftCols(m_m).transpose()*m_P*m_Ad);
    }
}

template<typename T>
void Lqr<T>::update_horizon_cost() {
    state_cost_t M;
    if (m_m == T::m) {
        M.noalias() = m_Ad + m_Bd*m_K;
        m_P = m_Q + m_K.transpose()*m_R*m_K + M.transpose()*m_P*M;
    } else {
        M.noalias() = m_Ad + m_Br.leftCols(m_m)*m_K.topRows(m_m);
        m_P = m_Q + m_K.topRows(m_m).transpose()*m_Rr.topLeftCorner(m_m, m_m)*m_K.topRows(m_m) + M.transpose()*m_P*M;
    }
}

template<typename T>
void Lqr<T>::set_control_mask() {
    // R must be positive definite. If any entry is 0.0, interpret as an
    // unavailable input and reduce the input matrix.
    m_mask = (m_R.diagonal().array() != 0.0).template cast<uint32_t>();
    m_m = m_mask.sum();
    //std::cout << "mask set: " << m_mask.transpose() << std::endl;
    //std::cout << "nonzero entries = " << m_m << std::endl;
    reduce_input_matrices();
}

template<typename T>
void Lqr<T>::reduce_input_matrices() {
    m_K.setZero();
    if (m_m == T::m) {
        m_Br.setZero();
        m_Rr.setZero();
        //std::cout << "reduce input matrices: skipping reduction" << std::endl;
    } else {
        for (unsigned int i = 0, j = 0; i < T::m; ++i) {
            if (m_mask[i]) {
                m_Br.col(j) = m_Bd.col(i);
                m_Rr.col(j++) = m_R.col(i);
            }
        }
        for (unsigned int i = 0, j = 0; i < T::m; ++i) {
            if (m_mask[i]) {
                m_Rr.row(j++) = m_Rr.row(i);
            }
        }
    }
    //std::cout << "initial B\n" << m_Bd << std::endl;
    //std::cout << "initial R\n" << m_R << std::endl;
    //std::cout << "reduced B [" << T::n << ", " << m_m << "]\n" << m_Br << std::endl;
    //std::cout << "reduced R [" << m_m << ", " << m_m << "]\n" << m_Rr << std::endl;
}


} // namespace controller
