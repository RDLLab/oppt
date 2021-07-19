/** @file RobotModel.hpp
 *
 * Contains RockSampleModel, which implements the core Model interface for the RockSample POMDP.
 */
#ifndef Robot_MODEL_HPP_
#define Robot_MODEL_HPP_

//#include <ios>                          // for ostream
//#include <memory>                       // for unique_ptr
//#include <string>                       // for string
//#include <utility>                      // for pair
//#include <vector>                       // for vector
//#include <random>

#include "RobotTransitionParameters.hpp"
#include "RobotState.hpp"
#include "RobotObservation.hpp"
#include "ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solvers/ABT/solver/abstract-problem/Action.hpp"            // for Action
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"       // for Observation
#include "solvers/ABT/solver/abstract-problem/State.hpp"       // for State
#include "solvers/ABT/solver/abstract-problem/ModelChange.hpp"

#include "solvers/ABT/solver/mappings/actions/enumerated_actions.hpp"
#include "solvers/ABT/solver/mappings/observations/enumerated_observations.hpp"

#include "solvers/ABT/solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solvers/ABT/solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model


#include "RobotAction.hpp"
#include "position_history.hpp"

#include "oppt/global.hpp"                     // for RandomGenerator
//#include "changes/EnvironmentChange.hpp"
#include "ContinuousRobotActionContainer.hpp"
#include "oppt/utils/include/Sampler.hpp"
#include "stepper/Gps2StepGenerator.hpp"

#include "solvers/ABT/ABTOptions.hpp"
#include "oppt/plugin/Plugin.hpp"
#include "mappings/enumeratedObservations.hpp"

using namespace oppt;

namespace abt
{
class ActionMapping;
class StatePool;
class DiscretizedPoint;
} /* namespace abt */

/** A namespace to hold the various classes used for the RockSample POMDP model. */
namespace robot
{
//class RobotMdpSolver;
class RobotState;
class RobotObservation;

class SimulationStepResult
{
public:
    SimulationStepResult() {

    }

    oppt::RobotStateSharedPtr currentState = nullptr;

    oppt::ActionSharedPtr action = nullptr;

    VectorFloat controlError;

    oppt::RobotStateSharedPtr resultingState = nullptr;

    oppt::ObservationSharedPtr observation = nullptr;

    std::shared_ptr<oppt::CollisionReport> collisionReport = nullptr;

    FloatType reward;

    bool terminal;
};


/** The implementation of the Model interface for the RockSample POMDP.
 *
 * See this paper http://arxiv.org/ftp/arxiv/papers/1207/1207.4166.pdf
 * for a description of the RockSample problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class RobotModel : public shared::ModelWithProgramOptions
{      
public:
    /** Constructs a new RobotModel instance with the given random number engine, and the
     * given set of configuration options.
     */
    RobotModel(RandomEnginePtr randGen, std::unique_ptr<ABTExtendedOptions> options);
    virtual ~RobotModel();
    _NO_COPY_OR_MOVE(RobotModel);


    /* --------------- The model interface proper ----------------- */
    virtual std::unique_ptr<abt::State> sampleAnInitState() override;

    virtual Model::StepResult generateStep(abt::State const& state,
                                           abt::Action const& action
                                          ) override;

    /** Generates a state uniformly at random. */
    virtual std::unique_ptr<abt::State> sampleStateUninformed() override;
    virtual bool isTerminal(abt::State const& state) override;
    virtual bool isValid(abt::State const& state) override;


    /* -------------------- Black box dynamics ---------------------- */
    virtual std::unique_ptr<abt::State> generateNextState(
        abt::State const& state,
        abt::Action const& action,
        abt::TransitionParameters const* /*tp*/) override;
    virtual std::unique_ptr<abt::Observation> generateObservation(
        abt::State const* /*state*/,
        abt::Action const& action,
        abt::TransitionParameters const* /*tp*/,
        abt::State const& nextState) override;
    virtual FloatType generateReward(
        abt::State const& state,
        abt::Action const& action,
        abt::TransitionParameters const* /*tp*/,
        abt::State const* /*nextState*/) override;

    virtual std::unique_ptr<abt::TransitionParameters> generateTransition(
        abt::State const& state,
        abt::Action const& action
    ) override;

    oppt::PropagationResultSharedPtr makeNextState(const oppt::RobotStateSharedPtr& robotState,
            const oppt::ActionSharedPtr& action);

    /* ---------------------- Basic customizations  ---------------------- */
    virtual FloatType getDefaultHeuristicValue(abt::HistoryEntry const* entry,
            abt::State const* state, abt::HistoricalData const* data) override;
            
    virtual std::unique_ptr<abt::StateIndex> createStateIndex() override;

    /* ----------------------- Methods for handling changes -------------- */
    virtual void applyChanges(std::vector<std::unique_ptr<abt::ModelChange>> const& changes,
                              abt::Solver* solver) override;

    virtual std::unique_ptr<abt::HistoricalData> createRootHistoricalData() override;

    virtual std::unique_ptr<abt::ActionPool> createActionPool(abt::Solver* solver) override;

    virtual FloatType makeReward(const oppt::PropagationResultSharedPtr& propagationResult);

    virtual std::unique_ptr<shared::RobotObservation> makeObservation(oppt::ActionSharedPtr& action,
            oppt::RobotStateSharedPtr& robotState);

    virtual std::unique_ptr<abt::ObservationPool> createObservationPool(abt::Solver* solver) override;

    virtual std::unique_ptr<abt::Serializer> createSerializer(abt::Solver* solver) override;

    void setRobotEnvironment(oppt::RobotEnvironment* robotEnvironment);

    void setHeuristicPlugin(const oppt::HeuristicPlugin* heuristicPlugin);

    std::unique_ptr<abt::Action> makeRobotAction(const oppt::ActionSharedPtr& opptAction);

    oppt::ActionSharedPtr getBaseAction(const abt::Action& action) const;

    oppt::RobotEnvironment* getRobotEnvironment() const;

    RandomEnginePtr getRandomEngine() const;
private:

    /** The RockSampleOptions instance associated with this model. */
    ABTExtendedOptions* options_;

    oppt::RobotEnvironment* robot_environment_;

    const oppt::RewardPlugin* rewardPlugin_;

    const oppt::HeuristicPlugin* heuristicPlugin_;

    RandomEnginePtr randGen_;    
    
    int seed = 123456789;
};
} /* namespace Robot */

#endif /* Robot_MODEL_HPP_ */
