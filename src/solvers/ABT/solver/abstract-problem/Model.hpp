/** @file Model.hpp
 *
 * Defines the Model class, which is an abstract class representing a POMDP model to be solved by
 * the solver. The core of the Model class is represented by a "black box" generative model which
 * is used to generate new steps in a simulation.
 *
 * In particular, generateStep() is the key method that defines the black box dynamics of the
 * POMDP - it takes in a state and action, and returns the resulting observation, reward, and
 * next state.
 *
 * Also of key importance is sampleAnInitState(), which samples an initial state according to
 * the initial belief distribution, and isTerminal(), which returns true iff the given state is
 * a terminal state.
 */
#ifndef SOLVER_MODEL_HPP_
#define SOLVER_MODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "oppt/global.hpp"                     // for RandomGenerator

#include "solvers/ABT/solver/abstract-problem/Action.hpp"        // for Action
#include "solvers/ABT/solver/abstract-problem/ModelChange.hpp"                    // for State
#include "solvers/ABT/solver/abstract-problem/State.hpp"                    // for State
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "oppt/options/Options.hpp"              // for Options
#include "solvers/ABT/solver/abstract-problem/TransitionParameters.hpp"

#include "solvers/ABT/solver/abstract-problem/heuristics/HeuristicFunction.hpp"
#include "solvers/ABT/solver/StateInfo.hpp"

namespace abt
{
class ActionPool;
class BeliefNode;
class EstimationStrategy;
class HistoricalData;
class HistoryCorrector;
class ObservationPool;
class SearchStrategy;
class SelectRecommendedActionStrategy;
class Serializer;
class Solver;
class StateIndex;
class StatePool;

/** An abstract class representing a black-box POMDP model for use by the ABT solver.
 *
 * The key methods are:
 * - sampleAnInitState() - samples a new state from the initial belief.
 * - isTerminal() - returns true iff the given state is terminal.
 * - generateStep() - the key method representing the generative model; basically, it does
 *          (s, a) => (o, r, s')
 * - applyChanges() - applies changes to the model, and interfaces with the Solver in order to
 *      determine how the policy is affected by the model changes.
 * - generateParticles() - these two methods define an explicit particle filtering approach, which
 *      allows the ABT algorithm to replenish its particles if the simple act of generating
 *      histories has not generated a sufficient number of particles for the current belief. The
 *      default implementation is quite general, but also relatively slow as it uses rejection
 *      sampling. Custom implementations should be provided for efficiency.
 * - getHeuristicFunction() - this defines a heuristic function that will be applied to the
 *      end of a history sequence that did not reach a terminal state. By default this is simply
 *      a function that returns zero.
 * - createActionPool() - this defines the way in which actions are mapped out inside the policy
 *      tree, and also which actions the ABT solver will attempt, and in which order.
 *
 * For the purposes of more subtle generation, ABT allows for a more incremental approach
 * to the generative model. In particular, we introduce optional "transition parameters", which
 * can serve the following key purposes:
 * - Encompassing any and all "randomness" that features in a state transition. This has two key
 *      advantages:
 *      - When changing histories, we can minimize the number of histories that "diverge" from the
 *          original path.
 *      - This can also be used for various variance reduction techniques, which are well
 *          documented in the literature.
 * - Storing intermediate information about the generative calculations that would otherwise be
 *      relatively expensive to calculate.
 *
 * The generative model is therefore further broken down into the following methods; some of these
 * parameters are optional and nullptr can be used for those values if they are not in use. In the
 * summary below, optional parameters are surrounded by square brackets [].
 * - generateTransition() -  (s, a) => [x]
 * - generateNextState() -   (s, a, [x]) => s'
 * - generateObservation() - ([s], a, [x], s') => o
 * - generateReward() -      (s, a, [x], [s']) => r
 */
class Model
{
public:
    /** Creates a new Model with the given name, random engine, and the given configuration
     * settings.
     *
     * Note that the options passed in will most typically be a subclass of the base Options class,
     * which allows for problem-specific configuration settings in a convenient way.
     */
    Model(std::string name, RandomGenerator* randGen, std::unique_ptr<oppt::Options> options);
    virtual ~Model() = default;
    _NO_COPY_OR_MOVE(Model);

    /* -------------------- Simple getters ---------------------- */
    /** Returns the random number generator used by this model. */
    RandomGenerator* getRandomGenerator() const;
    /** Returns the configuration options for this model. */
    oppt::Options const* getOptions() const;
    /** Returns the name of this problem. */
    std::string getName() const;


    /* --------------- The model interface proper ----------------- */
    /** Samples an initial state from the initial belief. */
    virtual std::unique_ptr<State> sampleAnInitState() = 0;
    /** Samples a state from a poorly-informed prior. This is used by the provided default
     * implementation of the second generateParticles() method.
     */
    virtual std::unique_ptr<State> sampleStateUninformed() = 0;
    /** Returns true iff the given state is terminal. */
    virtual bool isTerminal(State const& state) = 0;
    /** Returns true iff the given state is valid. */
    virtual bool isValid(State const& state) = 0;


    /* -------------------- Black box dynamics ---------------------- */
    /** Represents the results of a complete step in the model, including the next state,
     * observation, and reward, as well as any transition parameters used.
     *
     * For convenience, this also includes the action taken, and a boolean flag representing
     * whether or not the resulting next state is a terminal state.
     */
    struct StepResult {
        /** The action taken, which caused this result. */
        std::unique_ptr<Action> action = nullptr;
        /** The transition parameters, if any, describing the transition. */
        std::unique_ptr<TransitionParameters> transitionParameters = nullptr;
        /** The observation received. */
        std::unique_ptr<Observation> observation = nullptr;
        /** The resulting reward. */
        FloatType reward = 0;
        /** The next state */
        std::unique_ptr<State> nextState = nullptr;
        /** True iff the next state is terminal. */
        bool isTerminal = false;
    };

    /** Generates a full StepResult, including the next state, an observation, and the reward,
     * as well as any transition parameters used.
     *
     * For convenience, the action taken is also included in the result, as well as a flag for
     * whether or not the resulting next state is terminal.
     */
    virtual StepResult generateStep(
        State const& state,
        Action const& action	
    ) = 0;

    virtual void updateModel(StepResult& stepResult,
                             std::vector<State const*>& particles,
                             std::vector<std::vector<FloatType>>& particleColors);

    /** Generates the parameters for a next-state transition, if any are being used.
     *
     * This method is optional - the default implementation simply returns nullptr.
     */
    virtual std::unique_ptr<TransitionParameters> generateTransition(
        State const& state,
        Action const& action
    );

    /** Generates the next state, based on the state and action, and, if used,
     * the transition parameters.
     *
     * This method is only mandatory if you implement the applyChanges() method.
     */
    virtual std::unique_ptr<State> generateNextState(
        State const& state,
        Action const& action,
        TransitionParameters const* transitionParameters // optional
    );

    /** Generates an observation, given the action and resulting next state;
     * optionally, the previous state and the transition parameters can also be used.
     *
     * This method is only mandatory if you implement the applyChanges() method.
     */
    virtual std::unique_ptr<Observation> generateObservation(
        State const* state, // optional
        Action const& action,
        TransitionParameters const* transitionParameters, // optional
        State const& nextState
    );

    /** Returns the reward for the given state, action; optionally this also
     * includes transition parameters and the next state - if they aren't
     * being used it is OK to use nullptr for those inputs.
     *
     * This method is only mandatory if you implement the applyChanges() method.
     */
    virtual FloatType generateReward(
        State const& state,
        Action const& action,
        TransitionParameters const* transitionParameters, // optional
        State const* nextState // optional
    );


    /* -------------- Methods for handling model changes ---------------- */
    /** Applies a number of changes to the model, and (if provided) to the given solver.
     *
     * This is the core method that is used for the Model to inform the Solver of which states
     * are affected, and hence need to be updated in whichever history sequences they occur.
     * This should be done via the Solver's StatePool and StateIndex.
     *
     * Since handling of changes is not mandatory, this method does nothing by default. However,
     * if you do implement it, you must also implement
     * generateNextState(), generateObservation(), and generateReward().
     */
    virtual void applyChanges(std::vector<std::unique_ptr<ModelChange>> const& changes,
                              Solver* solver);

    /* ------------------- Pretty printing methods --------------------- */
    /** Draws the environment map (independent of the current state or belief) onto
     * the given output stream.
     *
     * Does nothing by default.
     */
    virtual void drawEnv(std::ostream& /*os*/);
    /** Draws the current belief and/or the actual current state in the context of the overall map
     * onto the given output stream.
     *
     * Does nothing by default.
     */
    virtual void drawSimulationState(BeliefNode const* belief, State const& state, std::ostream& os);


    /* ---------------------- Basic customizations  ---------------------- */
    /** Returns the function that approximates the value of a history entry based on the history
     * and/or an estimate using a single state.
     *
     * By default, this is simply a function that returns zero for every state.
     */
    virtual HeuristicFunction getHeuristicFunction();

    /** Returns a rollout action to be use, which can be based on the current state, the current
     * belief, and/or the history.
     *
     * By default, this method simply returns nullptr, because rollouts are not used at all by the
     * default search strategy - instead it simply uses the heuristic function.
     *
     * Providing the HistoryEntry allows access to the history sequence and belief associated with
     * the history entry, allowing for more intelligent POMDP-aware heuristics.
     *
     * Providing the HistoricalData allows for convenient history-based rollout policies that can
     * be efficiently calculated per belief node rather than requiring explicit calculation from
     * the entire history sequence.
     *
     * Providing the current state also allows for optimistic rollout strategies based on
     * perfect information.
     */
    virtual std::unique_ptr<Action> getRolloutAction(HistoryEntry const* entry, State const* state,
            HistoricalData const* data);

    /* ------- Customization of more complex solver functionality  --------- */
    // These are factory methods to allow the data structures used by ABT to be chosen in a
    // customizable way.

    /** Creates a StateIndex, which manages searching for states that have been used in a
     * StatePool.
     *
     * By default, this method uses an R*-Tree, as implemented in libspatialindex, in order to
     * allow range-based queries for the states.
     */
    virtual std::unique_ptr<StateIndex> createStateIndex() = 0;

    /** Creates a HistoryCorrector for this Model.
     *
     * By default, this simply makes an instance of DefaultHistoryCorrector, which should work for
     * pretty much all POMDP problems.
     */
    virtual std::unique_ptr<HistoryCorrector> createHistoryCorrector(Solver* solver);

    /** Creates an ActionPool, which manages actions and creates ActionMappings, for the given
     * solver.
     *
     * No default implementation is given, because there is no simple, general-purpose action
     * mapping class. This is because the way in which actions are selected within the search tree
     * depends strongly on the way in which the action space is specified.
     *
     * However, for most purposes the EnumeratedActionMapping class should work well. This requires
     * that the actions be enumerated [0, ..., nActions - 1], and that each action must be able to
     * return its action number.
     */
    virtual std::unique_ptr<ActionPool> createActionPool(Solver* solver) = 0;

    /** Creates an ObservationPool, which manages observations and creates ObservationMappings,
     * for the given solver.
     *
     * The default implementation is DiscreteObservationPool, which stores the observations for
     * each ActionNode in a hashtable. This should be effective for any problem with a discrete
     * observation space.
     */
    virtual std::unique_ptr<ObservationPool> createObservationPool(Solver* solver);

    /** Creates a search strategy for use by the given solver.
     *
     * The default strategy is simply a combination of UCB, in conjunction with applying the
     * default heuristic function implemented by this model to the resulting state as soon as
     * a new node in the policy tree is reached.
     *
     * Note that the default strategy simply uses a UCB coefficient of 1.0 - if you need to
     * change this coefficient value, simply override this method.
     */
    virtual std::unique_ptr<SearchStrategy> createSearchStrategy(Solver* solver);

    /** Creates an action recommendation strategy for use by the given solver.
     *
     * The default action returns simply the action with the maximal estimated Q-value.
     *
     */
    virtual std::unique_ptr<SelectRecommendedActionStrategy> createRecommendationSelectionStrategy(Solver* solver);

    /** Creates a strategy for estimating the value of belief nodes, for backprop, for the given
     * solver.
     *
     * The default approach is to simply take the average value of the estimated Q(s, a) values,
     * weighted by the number of visits for each action - this is equivalent to the average value
     * of all of the histories that go through this node.
     *
     * This is a general-purpose approach which should converge for any problem as long as it is
     * used with a search strategy that uses a UCB-like algorithm.
     */
    virtual std::unique_ptr<EstimationStrategy> createEstimationStrategy(Solver* solver);

    /** Creates the historical data for the root node.
     *
     * By default this returns a null pointer, as the HistoricalData interface is optional.
     */
    virtual std::unique_ptr<HistoricalData> createRootHistoricalData();

    /** Creates a Serializer for the given solver.
     *
     * By default this returns a null pointer. However, if you require serialization of the state
     * of an ABT solver, you must implement a custom Serializer which will properly serialize
     * the various clases you have implemented as part of your POMDP model.
     */
    virtual std::unique_ptr<Serializer> createSerializer(Solver* solver);

    virtual abt::StateInfo* sampleParticle(const std::vector<abt::StateInfo*>& stateInfos);

private:
    /** A string representing the name of this POMDP problem. */
    std::string problemName_;
    /** The random engine to use for generating state transitions and sampling. */
    RandomGenerator* randGen_;
    /** The configuration settings for this POMDP problem. */
    std::unique_ptr<oppt::Options> options_;
};
} /* namespace abt */

#endif /* SOLVER_MODEL_HPP_ */
