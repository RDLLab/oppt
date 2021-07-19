/** @file RolloutHeuristic.cpp
 *
 * Contains the implementation of the RolloutHeuristic class.
 */
#include "RolloutHeuristic.hpp"

#include <memory>

#include "solvers/ABT/solver/HistoryEntry.hpp"

#include "solvers/ABT/solver/abstract-problem/HistoricalData.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"

namespace abt {
RolloutHeuristic::RolloutHeuristic(Model *model, std::unique_ptr<StepGeneratorFactory> factory,
        HeuristicFunction heuristic) :
        model_(model),
        factory_(std::move(factory)),
        heuristic_(heuristic) {
}

FloatType RolloutHeuristic::getHeuristicValue(HistoryEntry const *entry,
        State const *state, HistoricalData const *data) {
    SearchStatus status = SearchStatus::UNINITIALIZED;
    std::unique_ptr<StepGenerator> generator = factory_->createGenerator(status,
            entry, state, data);
    FloatType value = 0.0;
    FloatType netDiscount = 1.0;
    FloatType discountFactor = 
    static_cast<oppt::ProblemEnvironmentOptions const*>(model_->getOptions())->discountFactor;

    Model::StepResult result = generator->getStep(entry, state, data);
    std::unique_ptr<State> currentState = state->copy();
    std::unique_ptr<HistoricalData> currentData = nullptr;
    while (result.action != nullptr) {
        value += netDiscount * result.reward;
        netDiscount *= discountFactor;

        currentState = result.nextState->copy();
        if (data != nullptr) {
            currentData = data->createChild(*result.action, *result.observation);
            data = currentData.get();
        }
        result = generator->getStep(nullptr, result.nextState.get(), data);
    }
    value += netDiscount * heuristic_(nullptr, currentState.get(), data);
    return value;
}

HeuristicFunction RolloutHeuristic::asFunction() {    
    return std::bind(&RolloutHeuristic::getHeuristicValue, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}
} /* namespace abt */
