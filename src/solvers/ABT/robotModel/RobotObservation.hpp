/** @file ManipulatorObservation.hpp
 *
 * Defines the ManipulatorObservation class, which represents an observation in the Manipulator POMDP.
 */
#ifndef Robot_OBSERVATION_HPP_
#define Robot_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solvers/ABT/solver/abstract-problem/DiscretizedPoint.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"

#include "oppt/global.hpp"                     // for RandomGenerator


#include "RobotState.hpp"
//#include "prob.hpp"

#include <boost/random.hpp>
#include "oppt/robotHeaders/Observation.hpp"

namespace shared {
/** A class representing an observation in the RockSample POMDP.
 *
 * This is represented by two boolean flags; one for whether or not any observation was made,
 * and one for whether the rock was seen as good or bad if an observation was, in fact, made.
 *
 * This class also implements abt::DiscretizedPoint so that the solver can use a simplistic
 * enumerated observation mapping approach (EnumeratedObservationPool) to store the possible
 * observations from each ActionNode.
 */
class RobotObservation : public abt::DiscretizedPoint {    

  public:
    /** Constructs a new observation. */
    RobotObservation(const oppt::ObservationSharedPtr &observation);
    
    virtual ~RobotObservation() = default;
    _NO_COPY_OR_MOVE(RobotObservation);

    std::unique_ptr<abt::Observation> copy() const override;
    FloatType distanceTo(abt::Observation const &otherObs) const override;
    bool equals(abt::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;
    
    virtual void serialize(std::ostream &os, const std::string prefix="") const override;

    long getBinNumber() const override;
    
    oppt::ObservationSharedPtr getOpptObservation() const;
    
  private:
    oppt::ObservationSharedPtr observation_;
  
};
} /* namespace manipulator */

#endif /* MANIPULATOR_OBSERVATION_HPP_ */
