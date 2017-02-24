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
         * Although model_t, state_t, input_t, measurement_t are declared in the Observer<T> class
         * and are inherited, the type aliases are dependent names as the base class (Observer)
         * is a template dependent on the template parameter of Oracle. In order to refer to
         * the type alias, we would need to specify 'typename Oracle::state_t', thus it is
         * easier to simply redeclare the type aliases.
         */
        using model_t = T;
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;

        Oracle(T& system);
        Oracle(T& system, const state_t& x0);

        virtual void reset() override;
        virtual void update_state(const input_t& u, const measurement_t& z) override;

    private:
        using Observer<T>::m_system;
        using Observer<T>::m_x;
};

} // namespace observer

#include "oracle.hh"
