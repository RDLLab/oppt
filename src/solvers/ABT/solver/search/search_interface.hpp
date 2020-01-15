/** @file search_interface.hpp
 *
 * Defines the interface for "searching" - i.e. generating new history sequences, both inside
 * and outside the policy tree.
 *
 * The core class is the SearchStrategy class, which must define a method "extendAndBackup", which
 * will extend a history sequence and backpropagate this new result within the policy tree.
 *
 * The provided implementation, BasicSearchStrategy, should be sufficient for most purposes. In
 * short, this implementation wraps a StepGeneratorFactory, which will be used to generate steps
 * in the sequence one at a time, and a heuristic, which will be applied to evaluate non-terminal
 * states that occur as the last entry in a sequence.
 */
#ifndef SOLVER_SEARCH_INTERFACE_HPP_
#define SOLVER_SEARCH_INTERFACE_HPP_

#include <memory>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/abstract-problem/heuristics/HeuristicFunction.hpp"

#include "SearchStatus.hpp"

#include "action-choosers/gps_choosers.hpp"


namespace abt {
class BeliefNode;
class HistoricalData;
class HistoryEntry;
class HistorySequence;
class Solver;

/** An interface for the basic search functionality.
 *
 * Implementing classes must define the extendAndBackup method, which should extend the given
 * history sequence and back up the effects of the newly added entries.
 */
class SearchStrategy {
public:
    SearchStrategy() = default;
    virtual ~SearchStrategy() = default;
    _NO_COPY_OR_MOVE(SearchStrategy);

    /** Extends and backs up the given HistorySequence, up to the given maximum depth in the
     * tree.
     *
     * Typically this method will be called with a new sequence with just the first entry; this is
     * essentially the same as creating a new sequence from scratch.
     *
     * However, if model changes are being used it also needs to be able to handle the case where
     * the sequence to extend already has several entries. Note that, in this case, the full backup
     * will only proceed along the newly generated entries, not the old ones.
     *
     * Backpropagation back to the root of the tree should be deferred (i.e. set aside for later).
     */
    virtual SearchStatus extendAndBackup(HistorySequence *sequence, long maximumDepth, Action *action=nullptr) = 0;
};

/** An interface for the action recommendation functionality.
 *
 * Implementing classes must define the getAction method, which should select an action to execute on a given belief,
 */
class SelectRecommendedActionStrategy {
public:
	SelectRecommendedActionStrategy() = default;
    virtual ~SelectRecommendedActionStrategy() = default;
    _NO_COPY_OR_MOVE(SelectRecommendedActionStrategy);

    /** Selects an action to execute during the simulation phase.
     */
    virtual std::unique_ptr<Action> getAction(const BeliefNode* belief) = 0;
};




/** An abstract class for generating new steps in a  history sequence, one step at a time.
 *
 * The core method is getStep(), which should return a new Model::StepResult (containing an
 * action, transition parameters, observation, reward, and a next state, as well as a flag
 * for whether or not the next state is terminal).
 *
 * If the generator runs out of new steps to generate it should simply return a StepResult with
 * a null action; the SearchStatus can also be used to specify the reason for this.
 */
class StepGenerator {
public:
    /** Creates a new StepGenerator, which will use the given SearchStatus reference in order
     * to inform its caller of the status of the search.
     */
    StepGenerator(SearchStatus &status);
    virtual ~StepGenerator() = default;

    /** Returns the result of an additional step in the simulation;
     * a null action ends this phase of simulation.
     */
    virtual Model::StepResult getStep(HistoryEntry const *entry, State const *state,
            HistoricalData const *data, Action *action=nullptr) = 0;

protected:
    /** A reference to the SearchStatus that will be used to inform callers. */
    SearchStatus &status_;
};


/** An interface factory class for generating new instances of the StepGenerator class.
 *
 * Since each StepGenerator instance is tied to a specific history sequence and state, it is
 * useful to have a factory class to specify a generic way in which StepGenerator instances will
 * be created.
 */
class StepGeneratorFactory {
public:
    StepGeneratorFactory() = default;
    virtual ~StepGeneratorFactory() = default;

    /** Creates a new StepGenerator instance, which will use the given SearchStatus reference
     * to store its search state.
     *
     * This factory method takes many arguments in order to allow it to be used generically to
     * create StepGenerators based on different arguments.
     * => The HistoryEntry* allows access to the associated belief and history sequence, which
     *      can be used for more intelligent belief-based search (e.g. UCB)
     * => The State* allows access to the specific state, which can be used to make state-based
     *      heuristics (e.g. the QMDP heuristic).
     * => The HistoricalData* allows access to history-based data, which allows the use of
     *      history-based heuristics.
     */
    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data) = 0;
};

/** A StepGenerator implementation that combines a sequence of "phases" into one. This can be used
 * to combine, for example, an in-tree UCB phase with an out-of-tree phase.
 */
class StagedStepGenerator: public StepGenerator {
public:
    /** Constructs a new StagedStepGenerator with the given sequence of factories.
     *
     * Each factory will, in turn, be used to generate an individual StepGenerator instance, and
     * each instance can then be used to generate steps as long as it is able to, until the
     * end of that phase (or termination of the entire sequence).
     *
     * The end of a phase occurs when one of the generators returns a null action in its StepResult.
     * If the SearchStatus is OUT_OF_STEPS and there are more generators left, the next generator
     * in the sequence will be queried. Otherwise, the entire history is finished.
     */
    StagedStepGenerator(SearchStatus &status,
            std::vector<std::unique_ptr<StepGeneratorFactory>> const &factories,
            HistoryEntry const *entry, State const *state, HistoricalData const *data);
    virtual ~StagedStepGenerator() = default;
    _NO_COPY_OR_MOVE(StagedStepGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry, State const *state,
            HistoricalData const *data, Action *action=nullptr) override;

private:
    /** The sequence of factories to use to generate the individual instances. */
    std::vector<std::unique_ptr<StepGeneratorFactory>> const &factories_;
    /** An iterator to store the current position in the factory sequence. */
    std::vector<std::unique_ptr<StepGeneratorFactory>>::const_iterator iterator_;
    /** The individual StepGenerator that is currently in use. */
    std::unique_ptr<StepGenerator> generator_;
};

/** An implementation of StepGeneratorFactory that creates an instance of StagedStepGenerator.
 *
 * Basically, this is just a factory that wraps a sequence of other factories, which will then
 * be used sequentially in order to generate steps over multiple phases.
 */
class StagedStepGeneratorFactory: public StepGeneratorFactory {
public:
    /** Constructs a new StagedStepGeneratorFactory which will use the given sequence of factories
     * in order to generate the StepGenerator instances used to search.
     */
    StagedStepGeneratorFactory(std::vector<std::unique_ptr<StepGeneratorFactory>> factories);
    virtual ~StagedStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(StagedStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data);

private:
    /** The sequence of factories to use in the search. */
    std::vector<std::unique_ptr<StepGeneratorFactory>> factories_;
};

/** A basic implementation of the SearchStrategy interface which should be sufficient for most
 * purposes.
 *
 * This is done by wrapping a StepGeneratorFactory, which will create StepGenerator instances,
 * which in turn will create steps for the sequence, one step at a time.
 *
 * This class also wraps a heuristic, which will be applied to evaluate non-terminal states
 * that occur as the last entry in a sequence.
 */
class BasicSearchStrategy: public SearchStrategy {
public:
    /** Creates a new BasicSearchStrategy which will be associated with the given solver, and
     * will use the given StepGeneratorFactory and the given heuristic.
     */
    BasicSearchStrategy(Solver *solver, std::unique_ptr<StepGeneratorFactory> factory,
            HeuristicFunction heuristic);
    virtual ~BasicSearchStrategy() = default;
    _NO_COPY_OR_MOVE(BasicSearchStrategy);

    /** The default implementation of extendAndBackup, used by default in the ABT algorithm. */
    virtual SearchStatus extendAndBackup(HistorySequence *sequence, long maximumDepth, Action *action=nullptr) override;
private:
    /** The associated solver. */
    Solver *solver_;
    /** The factory to use for generating sequence steps. */
    std::unique_ptr<StepGeneratorFactory> factory_;
    /** The heuristic to use to evalulate non-terminal states at the end of a sequence. */
    HeuristicFunction heuristic_;
};

/** An implementation for the action recommendation strategy.
 *
 * This simply maximises the Q-value.
 */
class MaxRecommendedActionStrategy: public SelectRecommendedActionStrategy {
public:
	MaxRecommendedActionStrategy() = default;
    virtual ~MaxRecommendedActionStrategy() = default;
    _NO_COPY_OR_MOVE(MaxRecommendedActionStrategy);

    /** Selects an action to execute during the simulation phase.
     */
    virtual std::unique_ptr<Action> getAction(const BeliefNode* belief);
};

/** An implementation for the action recommendation strategy with gps search.
 *
 * This simply maximises the Q-value of all active (within gps search) actions.
 */
class GpsMaxRecommendedActionStrategy: public SelectRecommendedActionStrategy {
public:
	GpsMaxRecommendedActionStrategy(const choosers::GpsMaxRecommendationOptions& options);
    virtual ~GpsMaxRecommendedActionStrategy() = default;
    _NO_COPY_OR_MOVE(GpsMaxRecommendedActionStrategy);

    /** Selects an action to execute during the simulation phase.
     */
    virtual std::unique_ptr<Action> getAction(const BeliefNode* belief);
private:
    choosers::GpsMaxRecommendationOptions options;
};



} /* namespace abt */

#endif /* SOLVER_SEARCH_INTERFACE_HPP_ */
