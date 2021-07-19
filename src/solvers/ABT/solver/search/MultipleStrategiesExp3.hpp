/** @file MultipleStrategiesExp3.hpp
 *
 * Provides an EXP3-based approach to combining multiple different search strategies together.
 */
#ifndef SOLVER_MULTIPLESTRATEGIESEXP3_HPP_
#define SOLVER_MULTIPLESTRATEGIESEXP3_HPP_

#include <ctime>

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "SearchStatus.hpp"
#include "search_interface.hpp"

namespace abt {
/** An implementation of the SearchStrategy interface that functions as a meta-strategy, wrapping
 * together a number of other strategies in one.
 *
 * When asked to extend a sequence, this strategy will sample one of the wrapped strategies and
 * use it. If one of the strategies fails to initialize properly this meta-strategy will simply
 * continue to sample from the other strategies until one of them succeeds, or all of them have
 * failed.
 *
 * The coefficients for sampling the strategies are determined by the EXP3 algorithm.
 */
class MultipleStrategiesExp3: public SearchStrategy {
public:
    /** Stores the information about a single strategy used within EXP3. */
    struct StrategyInfo {
        /** The ID number of the strategy (0, 1, 2, ...). */
        long strategyNo = 0;
        /** The strategy itself. */
        std::unique_ptr<SearchStrategy> strategy = nullptr;
        /** The amount of time spent on this strategy, in milliseconds. */
        FloatType timeSpent = 0.0;
        /** The current weight of this strategy; this is combined with other factors to give the
         * probability.
         */
        FloatType weight = 0.0;
        /** The probability of using this strategy (should add to 1.0). */
        FloatType probability = 0.0;
        /** The number of times this strategy has been used. */
        long numberOfTimesUsed = 0;
    };

    /** Constructs a new combination of multiple strategies, using EXP3.
     *
     * This will be associated with the given solver, an will own the given vector of strategies;
     * the given exploration coefficient determines the exploitation/exploration tradeoff between
     * different strategies.
     */
    MultipleStrategiesExp3(Solver *solver, FloatType strategyExplorationCoefficient,
            std::vector<std::unique_ptr<SearchStrategy>> strategies);
    virtual ~MultipleStrategiesExp3() = default;
    _NO_COPY_OR_MOVE(MultipleStrategiesExp3);

    /** Samples a new strategy to use from the set of strategies. Any strategy with an ID from
     * the given set of IDs will be excluded - this allows trying of multiple strategies
     * when one of them fails outright.
     */
    virtual StrategyInfo *sampleAStrategy(
            std::unordered_set<long> strategiesToExclude = std::unordered_set<long> { });
    /** Updates the weight for the given strategy based on the given results, i.e. the amount of
     * time used and the change in the root Q-value.
     */
    virtual void updateStrategyWeights(long strategyNo, FloatType timeUsed, FloatType deltaRootQValue);

    virtual SearchStatus extendAndBackup(HistorySequence *sequence, long maximumDepth, Action *action=nullptr);

private:
    /** The associated solver. */
    Solver *solver_;
    /** The model of the associated solver. */
    Model *model_;
    /** The exploration coefficient for exploring different strategies. */
    FloatType strategyExplorationCoefficient_;
    /** A vector of all of the strategies wrapped within this meta-strategy. */
    std::vector<StrategyInfo> strategies_;
};
} /* namespace abt */

#endif /* SOLVER_MULTIPLESTRATEGIESEXP3_HPP_ */
