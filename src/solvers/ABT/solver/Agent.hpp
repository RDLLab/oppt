/** @file Agent.hpp
 *
 * Contains the Agent class, which represents an actual agent embedded in an environment.
 *
 * This is a general class that should allow ABT to interface with external sensors; it is
 * also used by the Simulator class to test performance of the algorithm.
 */
#ifndef SOLVER_AGENT_HPP_
#define SOLVER_AGENT_HPP_

#include <memory>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"

namespace abt {
class BeliefNode;
class Solver;

/** Represents an agent embedded within an environment.
 *
 * The agent is tied to a solver, which in turn has an associated model of the environment and
 * uses the ABT algorithm to find a good policy for the agent to take.
 *
 * The agent represents its current state by associating it with a node in the solver's belief tree.
 *
 * Note that multiple agents can use the same solver, as the belief tree representation
 * is not tied to the state of a specific agent.
 */
class Agent {
public:
    /** Constructs a new agent associated with the given solver. The initial belief will start
     * at the root node of the solver. */
    Agent(Solver *solver);
    ~Agent() = default;
    _NO_COPY_OR_MOVE(Agent);

    /** Returns the solver being used by this agent. */
    Solver *getSolver() const;
    /** Returns the agent's current choice of action. */
    std::unique_ptr<Action> getPreferredAction() const;
    /** Returns the current belief of the agent (as a BeliefNode within the solver's belief tree). */
    BeliefNode *getCurrentBelief() const;

    /** Sets the current belief of this agent to the given BeliefNode. */
    void setCurrentBelief(BeliefNode *belief);
    /** Updates the belief of this agent based on an action and observation. */
    void updateBelief(Action const &action, Observation const &observation);

private:
    /** The solver used by this agent. */
    Solver *solver_;
    /** The belief node in the tree that represents this agent's current belief. */
    BeliefNode *currentBelief_;
};

} /* namespace abt */

#endif /* SOLVER_AGENT_HPP_ */

