/** @file RockSampleTextSerializer.hpp
 *
 * Contains text-based serialization methods for the core classes implementing RockSample, that is:
 * RockSampleChange, RockSampleState, RockSampleAction, and RockSampleObservation.
 */
#ifndef ROBOT_TEXTSERIALIZER_HPP_
#define ROBOT_TEXTSERIALIZER_HPP_

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

#include "solvers/ABT/solver/mappings/actions/enumerated_actions.hpp"
#include "solvers/ABT/solver/mappings/actions/continuous_actions.hpp"
#include "solvers/ABT/solver/mappings/observations/approximate_observations.hpp"
#include "RobotSerializer.hpp"

#include "position_history.hpp"
#include "oppt/global.hpp"
#include "ActionPool.hpp"

namespace abt
{
class Solver;
} /* namespace abt */

namespace robot
{

class RobotDiscreteActionTextSerializer: public shared::RobotSerializer,
    public abt::DiscretizedActionTextSerializer, public PositionDataTextSerializer
{
public:
    RobotDiscreteActionTextSerializer() = default;

    RobotDiscreteActionTextSerializer(abt::Solver* solver);

    virtual ~RobotDiscreteActionTextSerializer() = default;
    _NO_COPY_OR_MOVE(RobotDiscreteActionTextSerializer);

    void saveAction(abt::Action const* action, std::ostream& os) override;

    std::unique_ptr<abt::Action> loadAction(std::istream& is) override;

    virtual void saveActionMapping(abt::ActionMapping const& map,
                                   std::ostream& os) override;

    virtual std::unique_ptr<abt::ActionMapping> loadActionMapping(abt::BeliefNode* node,
            std::istream& is) override;

    virtual void loadActionMapping(abt::DiscretizedActionMap& discMap, std::istream& is) override;


private:
    abt::Solver* solver_;
};

class RobotContinuousActionTextSerializer: virtual public shared::RobotSerializer,
    virtual abt::ContinuousActionTextSerializer, public PositionDataTextSerializer
{
//typedef ManipulatorModel Model;
    typedef robot::ContinuousRobotAction::ConstructionData ConstructionData;

public:
    /** Creates a new RockSampleTextSerializer instance, associated with the given solver. */
    RobotContinuousActionTextSerializer() = default;

    RobotContinuousActionTextSerializer(abt::Solver* solver);

    virtual ~RobotContinuousActionTextSerializer() = default;

    _NO_COPY_OR_MOVE(RobotContinuousActionTextSerializer);

    void saveAction(abt::Action const* action, std::ostream& os) override;

    std::unique_ptr<abt::Action> loadAction(std::istream& is) override;

    void saveConstructionData(const ThisActionConstructionDataBase* baseData, std::ostream& os) override;

    std::unique_ptr<RobotContinuousActionTextSerializer::ThisActionConstructionDataBase> loadConstructionData(std::istream& is) override;

};

} /* namespace manipulator */

#endif /* MANIPULATOR_TEXTSERIALIZER_HPP_ */
