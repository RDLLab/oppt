#include "RobotModel.hpp"
#include "solvers/ABT/solver/indexing/RTree.hpp"
#include "solvers/ABT/solver/indexing/FlaggingVisitor.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/mappings/actions/enumerated_actions.hpp"
#include "solvers/ABT/solver/mappings/observations/enumerated_observations.hpp"
#include "mappings/approximateObservations.hpp"

#include "solvers/ABT/solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/StatePool.hpp"

#include "RobotAction.hpp"
#include "RobotTextSerializer.hpp"
#include <boost/timer.hpp>
//#include <boost/random/random_device.hpp>
#include "ActionPool.hpp"
#include "oppt/opptCore/ObservationReport.hpp"
#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/HeuristicInfo.hpp"
#include "oppt/robotHeaders/ObservationSpaceDiscretizer.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/robotModel/solver/StatePool.hpp"
#include "oppt/robotHeaders/ActionSpaceDiscretizer.hpp"


using std::cout;
using std::endl;

using namespace shared;

namespace robot
{

RobotModel::RobotModel(RandomEnginePtr randGen,
                       std::unique_ptr<ABTExtendedOptions> options) :
    shared::ModelWithProgramOptions("Robot", randGen.get(), std::move(options)),
    options_(const_cast<ABTExtendedOptions*>(static_cast<ABTExtendedOptions const*>(getOptions()))),
    robot_environment_(nullptr),
    heuristicPlugin_(nullptr),
    randGen_(randGen)
    
{

}

RobotModel::~RobotModel()
{
    
}

RandomEnginePtr RobotModel::getRandomEngine() const
{
    return randGen_;
}

void RobotModel::setRobotEnvironment(oppt::RobotEnvironment* robotEnvironment)
{
    robot_environment_ = robotEnvironment;
    std::unique_ptr<Gps2Parser> gps2Parser = std::make_unique<Gps2Parser>();
    std::unique_ptr<Gps2MaxRecommendedActionStrategyParser> rp = std::make_unique<Gps2MaxRecommendedActionStrategyParser>();
    gps2Parser->setDimensions(robot_environment_->getRobot()->getActionSpace()->getNumDimensions());
    rp->setDimensions(robot_environment_->getRobot()->getActionSpace()->getNumDimensions());

    registerGeneratorParser("gps2", std::move(gps2Parser));
    registerSelectRecommendedActionParser("gps2max", std::move(rp));

}

/* --------------- The model interface proper ----------------- */
std::unique_ptr<abt::State> RobotModel::sampleAnInitState()
{
    oppt::RobotStateSharedPtr initialState = robot_environment_->sampleInitialState();
    assert(initialState && "Initial state is NULL. Can't continue");
    std::unique_ptr<shared::RobotState> initState =
        std::make_unique<shared::RobotState>(initialState);
    initState->setWeight(1.0);
    return std::move(initState);
}

std::unique_ptr<abt::State> RobotModel::sampleStateUninformed()
{
    //RandomGenerator* randGen = getRandomGenerator();
    oppt::RobotStateSharedPtr randomState =
        robot_environment_->getRobot()->getStateSpace()->sampleUniform(randGen_);
    return std::make_unique<shared::RobotState>(randomState);
}

bool RobotModel::isTerminal(abt::State const& state)
{
    const oppt::RobotStateSharedPtr robotState = static_cast<shared::RobotState const&>(state).getOpptState();
    oppt::PropagationResultSharedPtr propRes(new PropagationResult());
    propRes->nextState = robotState;
    return robot_environment_->isTerminal(propRes);
}

bool RobotModel::isValid(abt::State const& state)
{
    oppt::RobotStateSharedPtr opptState = static_cast<shared::RobotState const&>(state).getOpptState();
    oppt::PropagationResultSharedPtr propRes(new PropagationResult());
    propRes->nextState = opptState;
    return robot_environment_->isValid(propRes);
}

oppt::ActionSharedPtr RobotModel::getBaseAction(const abt::Action& action) const
{
    if (options_->actionType == "discrete") {
        return static_cast<robot::DiscreteRobotAction const&>(action).getOpptAction();
    }

    return static_cast<robot::ContinuousRobotAction const&>(action).getOpptAction();
}

abt::Model::StepResult RobotModel::generateStep(abt::State const& state,
        abt::Action const& action)
{
    shared::RobotState const& robotState = static_cast<shared::RobotState const&>(state);
    oppt::RobotStateSharedPtr opptState =
        static_cast<shared::RobotState const&>(state).getOpptState();
    oppt::ActionSharedPtr opptAction = getBaseAction(action);

    abt::Model::StepResult result;
    result.action = action.copy();

    oppt::PropagationResultSharedPtr propagationResult =
        makeNextState(opptState, opptAction);
    std::unique_ptr<shared::RobotState> nextRobotState(new shared::RobotState(propagationResult->nextState));
    nextRobotState->setPropagationResult(propagationResult);

    result.observation = makeObservation(opptAction, propagationResult->nextState);
    result.reward = makeReward(propagationResult);
    result.isTerminal = robot_environment_->isTerminal(propagationResult);

    result.nextState = std::move(nextRobotState);
    result.transitionParameters =
        std::make_unique<shared::RobotTransitionParameters>(propagationResult);

    return result;
}

std::unique_ptr<abt::TransitionParameters> RobotModel::generateTransition(
    abt::State const& state,
    abt::Action const& action)
{
    WARNING("Generate transition");
    shared::RobotState const& robotState = static_cast<shared::RobotState const&>(state);
    oppt::RobotStateSharedPtr opptState =
        static_cast<shared::RobotState const&>(state).getOpptState();
    oppt::ActionSharedPtr opptAction = getBaseAction(action);
    oppt::PropagationResultSharedPtr propagationResult =
        makeNextState(opptState, opptAction);
    return std::make_unique<shared::RobotTransitionParameters>(propagationResult);

}

std::unique_ptr<abt::Action> RobotModel::makeRobotAction(const oppt::ActionSharedPtr& opptAction)
{
    if (options_->actionType == "discrete") {
        return std::make_unique<robot::DiscreteRobotAction>(opptAction);
    } else {
        return std::make_unique<robot::ContinuousRobotAction>(opptAction);
    }

}

oppt::RobotEnvironment* RobotModel::getRobotEnvironment() const
{
    return robot_environment_;
}

std::unique_ptr<abt::State> RobotModel::generateNextState(
    abt::State const& state,
    abt::Action const& action,
    abt::TransitionParameters const* tp)
{
    if (tp) {
        // We already generated a next state, no need to generate another one
        oppt::PropagationResultSharedPtr propagationResult =
            static_cast<const shared::RobotTransitionParameters*>(tp)->getPropagationResult();
        std::unique_ptr<shared::RobotState> nextRobotState(new shared::RobotState(propagationResult->nextState));
        nextRobotState->setPropagationResult(propagationResult);
        return std::move(nextRobotState);

    }

    oppt::RobotStateSharedPtr robotState =
        static_cast<shared::RobotState const&>(state).getOpptState();
    oppt::ActionSharedPtr robotAction = getBaseAction(action);
    //RobotAction const& robotAction = (static_cast<robot::RobotAction const&>(action));
    oppt::PropagationResultSharedPtr propagationResult =
        makeNextState(robotState, robotAction);
    std::unique_ptr<shared::RobotState> nextRobotState(new shared::RobotState(propagationResult->nextState));
    nextRobotState->setPropagationResult(propagationResult);

    return std::move(nextRobotState);
}

oppt::PropagationResultSharedPtr RobotModel::makeNextState(const oppt::RobotStateSharedPtr& robotState,
        const oppt::ActionSharedPtr& action)
{
    oppt::PropagationRequestSharedPtr propagationRequest(new oppt::PropagationRequest());
    propagationRequest->currentState = robotState;
    propagationRequest->action = action.get();
    propagationRequest->allowCollisions = options_->allowCollisions;

    oppt::PropagationResultSharedPtr propagationResult =
        robot_environment_->getRobot()->propagateState(propagationRequest);
    return propagationResult;
}

std::unique_ptr<abt::Observation> RobotModel::generateObservation(
    abt::State const* /*state*/, abt::Action const& action,
    abt::TransitionParameters const* /*tp*/, abt::State const& nextState)
{
    oppt::RobotStateSharedPtr robotState = static_cast<shared::RobotState const&>(nextState).getOpptState();
    oppt::ActionSharedPtr robotAction = getBaseAction(action);
    return makeObservation(robotAction, robotState);
}

std::unique_ptr<shared::RobotObservation> RobotModel::makeObservation(oppt::ActionSharedPtr& action,
        oppt::RobotStateSharedPtr& robotState)
{
    ObservationRequestSharedPtr observationRequest = std::make_shared<ObservationRequest>();
    observationRequest->currentState = robotState;
    observationRequest->action = action.get();
    ObservationResultSharedPtr observationResult =
        robot_environment_->getRobot()->makeObservationReport(observationRequest);
    ObservationSharedPtr observation = observationResult->observation;
    return std::make_unique<shared::RobotObservation>(observation);
}

FloatType RobotModel::generateReward(
    abt::State const& state,
    abt::Action const& action,
    abt::TransitionParameters const* tp,
    abt::State const* nextState)
{
    if (!tp) {
        // This should never happen
        ERROR("Transition parameters are zero");
    }
    oppt::PropagationResultSharedPtr propagationResult =
        static_cast<const shared::RobotTransitionParameters*>(tp)->getPropagationResult();
    return makeReward(propagationResult);
}

FloatType RobotModel::makeReward(const oppt::PropagationResultSharedPtr& propagationResult)
{
    return robot_environment_->getReward(propagationResult);
}

std::unique_ptr<abt::ActionPool> RobotModel::createActionPool(abt::Solver* /*solver*/)
{
    std::unique_ptr<abt::ActionPool> actionPool;
    oppt::ActionSpaceSharedPtr actionSpace = robot_environment_->getRobot()->getActionSpace();
    if (!actionSpace) {
        oppt::ERROR("RobotModel: createActionPool(): action space is null!!!");
    }
    if (options_->actionType == "discrete") {
        auto actionSpaceDiscretizer = actionSpace->getActionSpaceDiscretizer();
        if (!actionSpaceDiscretizer) {
            actionSpaceDiscretizer = std::make_shared<oppt::ActionSpaceDiscretizer>(actionSpace);
            if (options_->actionDiscretization.size() > 0) {
                actionSpaceDiscretizer = std::make_unique<CustomActionSpaceDiscretizer>(actionSpace, options_->actionDiscretization);
            }
        }

        std::vector<oppt::ActionSharedPtr> allOpptActions =
            actionSpaceDiscretizer->getAllActionsInOrder(options_->numInputStepsActions);
        unsigned int numActions = allOpptActions.size();
        LOGGING("Planning with " + std::to_string(numActions) + " actions");
        std::vector<std::unique_ptr<abt::DiscretizedPoint>> allActions(numActions);
        for (size_t i = 0; i < numActions; i++) {
            allActions[i] = std::make_unique<robot::DiscreteRobotAction>(allOpptActions[i]);
        }

        actionPool = std::make_unique<oppt::RobotEnumeratedActionPool>(this,
                     options_->numInputStepsActions,
                     numActions,
                     std::move(allActions));
    }

    else if (options_->actionType == "continuous") {
        oppt::LOGGING("RobotModel: Create action pool for continuous action type");
        oppt::ActionSpaceSharedPtr actionSpace = robot_environment_->getRobot()->getActionSpace();
        actionPool = std::make_unique<shared::ContActionPool>(actionSpace);
    } else {
        ERROR("Action type not recognized. Must be 'discrete' or 'continuous'");
    }

    return std::move(actionPool);
}

std::unique_ptr<abt::ObservationPool> RobotModel::createObservationPool(
    abt::Solver* solver)
{
    std::unique_ptr<abt::ObservationPool> obsPool;
    oppt::ObservationSpaceSharedPtr observationSpace = robot_environment_->getRobot()->getObservationSpace();
    if (options_->observationType == "discrete") {
        oppt::ObservationSpaceDiscretizer discretizer(observationSpace);
        std::vector<ObservationSharedPtr> allObservationsInOrder =
            discretizer.getAllObservationsInOrder(options_->numInputStepsObservations);
        std::vector<std::unique_ptr<abt::DiscretizedPoint>> allObservations(allObservationsInOrder.size());
        for (size_t i = 0; i != allObservationsInOrder.size(); ++i) {
            allObservations[i] = std::make_unique<shared::RobotObservation>(allObservationsInOrder[i]);
        }

        obsPool = std::make_unique<EnumeratedABTObservationPool>(solver, std::move(allObservations));
        return obsPool;
    } else if (options_->observationType == "continuous") {
        std::unique_ptr<abt::ObservationPool> ptr =
            std::make_unique<oppt::ApproximateABTObservationPool>(solver, options_->maxObservationDistance);
        return std::move(ptr);
    } else {
        ERROR("Observation type not recognized. Must be 'discrete' or 'continuous'");
    }

    return nullptr;
}

/* -------------- Methods for handling model changes ---------------- */
void RobotModel::applyChanges(std::vector<std::unique_ptr<abt::ModelChange>> const& changes,
                              abt::Solver* solver)
{
    if (robot_environment_->isExecutionEnvironment())
        return;

    cout << "RobotModel: Applying " << changes.size() << " changes" << endl;
    abt::OPPTStatePool* pool = nullptr;
    if (solver) {
        pool = static_cast<abt::OPPTStatePool*>(solver->getStatePool());
    }

    if (pool) {
        for (auto it = pool->begin(); it != pool->end(); ++it) {
            abt::StateInfo* info = it->second.get();
            auto propagationResult =
                static_cast<const shared::RobotState*>(info->getState())->getPropagationResult();
            if (propagationResult &&
                    propagationResult->collisionReport &&
                    propagationResult->collisionReport->collides) {
                oppt::RobotStateSharedPtr opptState =
                    static_cast<const shared::RobotState*>(info->getState())->getOpptState();
                oppt::PropagationResultSharedPtr propRes(new PropagationResult());
                propRes->nextState = opptState;
                if (!robot_environment_->isValid(propRes)) {
                    pool->setChangeFlags(info, abt::ChangeFlags::DELETED);
                }
            }
        }
    }

    return;
}

void RobotModel::setHeuristicPlugin(const oppt::HeuristicPlugin* heuristicPlugin)
{
    heuristicPlugin_ = heuristicPlugin;
}

FloatType RobotModel::getDefaultHeuristicValue(abt::HistoryEntry const* entry,
        abt::State const* state, abt::HistoricalData const* data)
{
    if (!heuristicPlugin_)
        return 0.0;
    oppt::HeuristicInfo* heuristicInfo(new oppt::HeuristicInfo());
    oppt::RobotStateSharedPtr robotState = static_cast<const shared::RobotState*>(state)->getOpptState();
    //oppt::ActionSharedPtr robotAction = static_cast<shared::RobotAction const *>(entry->getAction());

    heuristicInfo->currentState = static_cast<const shared::RobotState*>(state)->getOpptState();
    heuristicInfo->timeout = options_->heuristicTimeout;
    heuristicInfo->discountFactor = options_->discountFactor;
    if (data) {
        heuristicInfo->currentStep = static_cast<HistEntry const*>(data)->getCurrentStep();
        heuristicInfo->action = static_cast<HistEntry const*>(data)->getParentAction().get();
    }


    FloatType reward = heuristicPlugin_->getHeuristicValue(heuristicInfo);
    delete heuristicInfo;
    return reward;
}

std::unique_ptr<abt::StateIndex> RobotModel::createStateIndex()
{
    return std::make_unique<abt::RTree>(robot_environment_->getRobot()->getStateSpace()->getNumDimensions());
}

std::unique_ptr<abt::HistoricalData> RobotModel::createRootHistoricalData()
{
    oppt::RobotStateSharedPtr initialRobotState = robot_environment_->sampleInitialState();
    if (!initialRobotState) {
        oppt::ERROR("RobotModel: createRootHistoricalData: Initial state is NULL");
    }

    std::unique_ptr<abt::HistoricalData> histEntry(new HistEntry());
    return std::move(histEntry);
}

std::unique_ptr<abt::Serializer> RobotModel::createSerializer(abt::Solver* solver)
{
    std::unique_ptr<abt::Serializer> serializer;
    if (options_->actionType == "discrete") {
        serializer = std::make_unique<RobotDiscreteActionTextSerializer>(solver);
    } else if (options_->actionType == "continuous") {
        serializer = std::make_unique<RobotContinuousActionTextSerializer>(solver);
    }

    serializer->setSolver(solver);
    return serializer;

    assert(false && "Error: actionType not recognized");

}

} /* namespace rocksample */
