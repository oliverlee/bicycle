#pragma once
#include <Eigen/Core>
#include "types.h"
#include "discrete_linear.h"

namespace observer {
using real_t = model::real_t;

/* This class cannot be instantiated and does not allow polymorphic deletion through a base pointer.*/
class ObserverBase {
    protected:
        ~ObserverBase() { }
};

template <typename T>
class Observer : private ObserverBase {
    static_assert(std::is_base_of<model::DiscreteLinearBase, T>::value, "Invalid template parameter type for Observer");

    public:
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using measurement_t = typename T::output_t;

        virtual void reset() = 0;
        virtual void update_state(const input_t& u, const measurement_t& z) = 0;

        virtual void set_state(const state_t& x) = 0;

        virtual T& system() const = 0;
        virtual real_t dt() const = 0;
        virtual const state_t& state() const = 0;
};

} // namespace observer
