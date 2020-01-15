/** @file TransitionParameters.hpp
 *
 * An abstract base class for transition parameters, which allow intermediate generation of
 * steps in a sequence. If used, the idea is that the transition parameters can work as follows:
 * (s, a) => T => (o, r, s').
 *
 * This is useful when model changes need to be applied, as the transition parameters can store
 * values such as sampled random numbers or intermediate calculations, allowing the transitions
 * to be recalculated more efficiently. Also, not having to re-randomize means that unnecessary
 * divergence of histories due to resampling can be avoided.
 */
#ifndef SOLVER_TRANSITIONPARAMETERS_HPP_
#define SOLVER_TRANSITIONPARAMETERS_HPP_
#include <ostream>

namespace abt {

/** An abstract base to store transitional values for a generative model.
 *
 * This allows for intermediate values between a state-action pair to the subsequent next state,
 * observation and reward to be kept.
 *
 * The base class has no mandatory methods, because it is up to the Model to decide what to store;
 * the model can also choose not to use them at all by using null pointers.
 */
class TransitionParameters {
public:
    TransitionParameters() = default;
    virtual ~TransitionParameters() = default;

    /** Prints the parameters in a human-readable way [optional]. */
    virtual void print(std::ostream &/*os*/) const {};
    
    virtual void serialize(std::ostream &/*os*/, const std::string prefix="") const {}
};

/** Uses the virtual print method to output HistoricalData to a stream. */
inline std::ostream &operator<<(std::ostream &os,
        TransitionParameters const &tp) {
    tp.print(os);
    return os;
}

} /* namespace abt */

#endif /* SOLVER_TRANSITIONPARAMETERS_HPP_ */
