#pragma once
#include "observer.h"

namespace observer {

/*
 * This template class implements an observer that simply stores and returns
 * the system model state.
 */
template <typename T>
class Oracle final : public Observer<T> {
    public:
        /*
         * Although state_t, input_t, measurement_t are declared in the Observer<T> class
         * and are inherited, the type aliases are dependent names as the base class (Observer)
         * is a template dependent on the template parameter of Oracle. In order to refer to
         * the type alias, we would need to specify 'typename Oracle::state_t', thus it is
         * easier to simply redeclare the type aliases.
         */
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;

        Oracle(T& system);
        Oracle(T& system, const state_t& x0);

        virtual void reset() override;
        virtual void update_state(const input_t& u, const measurement_t& z) override;

        virtual void set_state(const state_t& x) override;

        virtual T& system() const override;
        virtual real_t dt() const override;
        virtual const state_t& state() const override;

    private:
        // TODO: consolidate these common fields in the Observer base class
        T& m_system;
        state_t m_state;
};

template <typename T>
inline void Oracle<T>::set_state(const state_t& x) {
    m_state = x;
}

template <typename T>
inline T& Oracle<T>::system() const {
    return m_system;
}

template <typename T>
inline real_t Oracle<T>::dt() const {
    return m_system.dt();
}

template <typename T>
inline const typename Oracle<T>::state_t& Oracle<T>::state() const {
    return m_state;
}

} // namespace observer

#include "oracle.hh"
