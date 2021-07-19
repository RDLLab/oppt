/** @file Model.cpp
 *
 * Contains default implementations for some of the methods of Model.
 *
 * These should be overridden as needed, in order to further customize the ABT functionality.
 */
#include "Model.hpp"

#include <functional>

#include "solvers/ABT/solver/cached_values.hpp"
#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"

#include "Action.hpp"        // for Action
#include "HistoricalData.hpp"
#include "State.hpp"                    // for State
#include "Observation.hpp"              // for Observation
#include "TransitionParameters.hpp"
#include "heuristics/HeuristicFunction.hpp"

#include "solvers/ABT/solver/belief-estimators/estimators.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionPool.hpp"
#include "solvers/ABT/solver/mappings/observations/discrete_observations.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationMapping.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationPool.hpp"

#include "solvers/ABT/solver/indexing/StateIndex.hpp"
#include "solvers/ABT/solver/indexing/RTree.hpp"

#include "solvers/ABT/solver/changes/DefaultHistoryCorrector.hpp"
#include "solvers/ABT/solver/changes/HistoryCorrector.hpp"

#include "solvers/ABT/solver/search/search_interface.hpp"
#include "solvers/ABT/solver/search/steppers/ucb_search.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"

using std::cout;
using std::endl;

namespace abt {
Model::Model(std::string problemName, RandomGenerator *randGen, std::unique_ptr<oppt::Options> options) :
        problemName_(problemName),
        randGen_(randGen),
        options_(std::move(options)) {
}

/* -------------------- Simple getters ---------------------- */
RandomGenerator *Model::getRandomGenerator() const {
    return randGen_;
}

oppt::Options const *Model::getOptions() const {
    return options_.get();
}

void Model::updateModel(StepResult &stepResult, 
                        std::vector<State const *> &particles,
                        std::vector<std::vector<FloatType>> &particleColors) {

}

/* -------------------- Black box dynamics ---------------------- */
// The more detailed methods are optional.

std::unique_ptr<TransitionParameters> Model::generateTransition(
        State const &/*state*/,
        Action const &/*action*/) {
    return nullptr;
}

std::unique_ptr<State> Model::generateNextState(
        State const &/*state*/,
        Action const &/*action*/,
        TransitionParameters const */*transitionParameters*/ // optional
        ) {
    return nullptr;
}

std::unique_ptr<Observation> Model::generateObservation(
          State const */*state*/, // optional
          Action const &/*action*/,
          TransitionParameters const */*transitionParameters*/, // optional
          State const &/*nextState*/
          ) {
    return nullptr;
}

FloatType Model::generateReward(
          State const &/*state*/,
          Action const &/*action*/,
          TransitionParameters const */*transitionParameters*/, // optional
          State const */*nextState*/ // optional
          ) {
    return 0;
}


/* -------------- Methods for handling model changes ---------------- */
// Default = no changes.
void Model::applyChanges(std::vector<std::unique_ptr<ModelChange>> const &/*changes*/,
        Solver */*solver*/) {
}

/* --------------- Pretty printing methods ----------------- */
void Model::drawEnv(std::ostream &/*os*/) {
    // Default = do nothing.
}
void Model::drawSimulationState(BeliefNode const */*belief*/, State const &/*state*/,
        std::ostream &/*os*/) {
    // Default = do nothing.
}

/* ---------------------- Basic customizations  ---------------------- */
/** The default implementation simply returns zero. */
HeuristicFunction Model::getHeuristicFunction() {
    return [] (abt::HistoryEntry const *, abt::State const *,
            abt::HistoricalData const *) {
        return 0;
    };
}

/** Optional; not implemented. */
std::unique_ptr<Action> Model::getRolloutAction(HistoryEntry const */*entry*/,
        State const */*state*/, HistoricalData const */*data*/) {
    return nullptr;
}

/* ------- Customization of more complex solver functionality  --------- */
std::unique_ptr<HistoryCorrector> Model::createHistoryCorrector(Solver *solver) {
    // Create a DefaultHistoryCorrector.
    return std::make_unique<DefaultHistoryCorrector>(solver, getHeuristicFunction());
}

std::unique_ptr<ObservationPool> Model::createObservationPool(Solver *solver) {
    // Create a DiscreteObservationPool
    return std::make_unique<DiscreteObservationPool>(solver);
}

std::unique_ptr<SearchStrategy> Model::createSearchStrategy(Solver *solver) {
    // Create a basic search strategy using UCB with a coefficient of 1.0, and the default
    // heuristic function for this model.
    return std::make_unique<BasicSearchStrategy>(solver,
            std::make_unique<UcbStepGeneratorFactory>(solver, 1.0),
            getHeuristicFunction());
}

std::unique_ptr<SelectRecommendedActionStrategy> Model::createRecommendationSelectionStrategy(Solver */*solver*/) {
    // Create a basic search strategy using max the Q-value function
    return std::make_unique<MaxRecommendedActionStrategy>();
}

std::unique_ptr<EstimationStrategy> Model::createEstimationStrategy(Solver */*solver*/) {
    // Returns an estimation strategy using a simple average.
    return std::make_unique<EstimationFunction>(estimators::average);
}

std::unique_ptr<HistoricalData> Model::createRootHistoricalData() {
    // Optional; not implemented.
    return nullptr;
}

std::unique_ptr<Serializer> Model::createSerializer(Solver */*solver*/) {
    // Optional; not implemented.
    return nullptr;
}

abt::StateInfo * Model::sampleParticle(const std::vector<abt::StateInfo *> &stateInfos) {
    RandomGenerator *randGen = getRandomGenerator();
    long index = std::uniform_int_distribution<long>(0, stateInfos.size() - 1)(*randGen);
    return stateInfos[index];
}
} /* namespace abt */
