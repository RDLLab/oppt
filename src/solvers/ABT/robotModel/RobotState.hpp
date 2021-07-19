#ifndef Robot_STATE_HPP_
#define Robot_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include <cmath>
#include "solvers/ABT/solver/abstract-problem/State.hpp"             // for State
#include "solvers/ABT/solver/abstract-problem/VectorState.hpp"             // for State
#include "oppt/robotHeaders/RobotState.hpp"
//#include <worldstateserializer/worldStateSerializer.hpp>

using namespace oppt;

namespace shared
{
/** A class representing a state in the RockSample POMDP.
 *
 * The state contains the position of the robot, as well as a boolean value for each rock
 * representing whether it is good (true => good, false => bad).
 *
 * This class also implements abt::VectorState in order to allow the state to be easily
 * converted to a vector<FloatType>, which can then be used inside the standard R*-tree implementation
 * of StateIndex to allow spatial lookup of states.
 */
class RobotState : public abt::VectorState
{
    friend class RobotTextSerializer;
public:
    /** Constructs a new RockSampleState with the given robot position, and the given goodness states
     * for all of the rocks.
     */
    RobotState(const oppt::RobotStateSharedPtr &robotState);

    virtual ~RobotState() = default;

    std::unique_ptr<abt::State> copy() const override;
    FloatType distanceTo(abt::State const& otherState) const override;
    
    bool equals(abt::State const& otherState) const override;

    std::size_t hash() const override;
    
    void print(std::ostream& os) const override;

    virtual void serialize(std::ostream& os, const std::string prefix="") const override;   

    void setWeight(const FloatType weight) const;

    FloatType getWeight() const;
    
    const oppt::RobotStateSharedPtr getOpptState() const;
    
    VectorFloat asVector() const override;
    
    void setPropagationResult(const oppt::PropagationResultSharedPtr &propagationResult);
    
    const oppt::PropagationResultSharedPtr getPropagationResult() const;

private:
    const oppt::RobotStateSharedPtr robotState_;
    
    // The PropagationResult this state origined from;
    oppt::PropagationResultSharedPtr propagationResult_;
    
    VectorFloat state_vector_;

    int roundingPrecision_ = 6;

    FloatType round_(const FloatType& value, const int& precision) const;

};
} /* namespace manipulator */

// We define a hash function directly in the std namespace.
namespace std
{
/** A struct in the std namespace to define a standard hash function for the
 * RockSampleState class.
 */
template<> struct hash<shared::RobotState> {
    /** Returns the hash value for the given RockSampleState. */
    std::size_t operator()(shared::RobotState const& state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* MANIPULATORSTATE_HPP_ */
