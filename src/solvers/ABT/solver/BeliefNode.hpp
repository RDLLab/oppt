/** @file BeliefNode.hpp
 *
 * Contains the BeliefNode class, which represents a single belief node within the belief tree.
 *
 * The core data stored by a belief node is the set of particles (history entries) associated with
 * this node, as well as an ActionMapping and a back-pointer.
 */
#ifndef SOLVER_BELIEFNODE_HPP_
#define SOLVER_BELIEFNODE_HPP_

#include <functional>
#include <map>                          // for map, map<>::value_compare
#include <memory>                       // for unique_ptr
#include <set>
#include <utility>                      // for pair

#include "oppt/global.hpp"                     // for RandomGenerator
#include "solvers/ABT/solver/RandomAccessSet.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"                   // for Action
#include "solvers/ABT/solver/abstract-problem/HistoricalData.hpp"
#include "solvers/ABT/solver/abstract-problem/State.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"

namespace oppt
{
class BeliefTree;
class ABTSolver;
}

namespace solvers {
class ABT;
}

namespace abt
{
class BaseCachedValue;
template <typename T> class CachedValue;

class ActionMapping;
class ActionNode;
class HistoricalData;
class HistoryEntry;
class ObservationMappingEntry;
class Solver;
class OPPTStatePool;

/** Represents a single node in a belief tree.
 *
 * The key functionality is a set of all the particles associated with this belief node, where
 * each particle is a pointer to a HistoryEntry.
 *
 * Additionally, a belief node owns an ActionMapping, which stores the actions that have been
 * taken from this belief, as well as their associated statistics and subtrees.
 *
 * The belief node can also store a vector of cached values, which is convenient if you want
 * to cache values that are derived from the contents of the belief via a relatively expensive
 * calculation.
 *
 * This caching is also particularly useful for incremental updates - the cached value can be
 * compared to its new value after recalculation, and then the change in value can be easily
 * backpropagated.
 */
class BeliefNode
{
public:
    friend class ActionNode;
    friend class BeliefTree;
    friend class HistoryEntry;
    friend class Solver;
    friend class TextSerializer;
    friend class OPPTStatePool;
    friend class oppt::BeliefTree;
    friend class oppt::ABTSolver;
    friend class solvers::ABT;

    /** Constructs a new belief node with no ID, and no parent entry, that will belong to the
     * given solver.
     */
    BeliefNode(Solver* solver);
    /** Constructs a new belief node with no ID (-1), with the given mapping entry as
     * its parent, and which will belong to the given solver.
     */
    BeliefNode(ObservationMappingEntry* parentEntry, Solver* solver);
    /** Constructs a new belief node with the given ID,with the given mapping entry as its parent,
     * and which will belong to the given solver.
     */
    BeliefNode(long id, ObservationMappingEntry* parentEntry, Solver* solver);

    // Default destructor; copying and moving disallowed!
    virtual ~BeliefNode();
    _NO_COPY_OR_MOVE(BeliefNode);

    virtual void clear();

    /* ---------------- Useful calculations ------------------ */
    /** Calculates the distance between this belief node and another by
     * calculating the average pairwise distance between the individual
     * particles.
     */
    FloatType distL1Independent(BeliefNode* b) const;

    /* -------------------- Simple getters ---------------------- */
    /** Returns the id of this node. */
    long getId() const;
    /** Returns the depth of this node in the tree (0 = root). */
    long getDepth() const;
    /** Returns the number of particles in this node. */
    long getNumberOfParticles() const;
    /** Returns the number of sequences starting from this node. */
    long getNumberOfStartingSequences() const;
    /** Returns a vector containing all of the states contained in node. */
    std::vector<State const*> getStates() const;

    std::vector<State const*> getNonTerminalStates() const;

    /* -------------------- Tree-related getters  ---------------------- */
    /** Returns the action mapping for this node. */
    ActionMapping* getMapping() const;
    /** Returns the parent entry for this node. */
    ObservationMappingEntry* getParentEntry() const;
    /** Returns the history-derived information for this node. */
    HistoricalData* getHistoricalData() const;
    /** Returns the parent action node of this belief. */
    ActionNode* getParentActionNode() const;
    /** Returns the parent belief of this belief. */
    BeliefNode* getParentBelief() const;
    /** Returns the last observation received before this belief. */
    std::unique_ptr<Observation> getLastObservation() const;
    /** Returns the last action taken before this belief. */
    std::unique_ptr<Action> getLastAction() const;
    /** Returns the belief node child corresponding to the given action and
     * observation
     */
    BeliefNode* getChild(Action const& action, Observation const& obs) const;

    /* ----------------- Management of cached values ------------------- */
    /** Adds a value to be cached by this belief node. */
    BaseCachedValue* addCachedValue(std::unique_ptr<BaseCachedValue> value);
    /** Removes a value cached by this belief node. */
    void removeCachedValue(BaseCachedValue* value);

    /* ------------ Value estimation and action selection -------------- */
    /** Sets the way in which the value for this belief node will be calculated. */
    void setValueEstimator(CachedValue<FloatType>* qEstimator);
    /** Returns the recommended action to take from this node. */
    std::unique_ptr<Action> getRecommendedAction() const;
    /** Returns the current cached value */
    FloatType getCachedValue() const;
    /** Recalculates the cached value for this belief node. */
    void recalculateValue();


    /* -------------------- Core tree-related methods  ---------------------- */
    /** Adds a child for the given action and observation, or returns a pre-existing one if it
     * already existed.
     *
     * The belief node will also be added to the flattened node vector of the policy tree, as
     * this is done by the BeliefNode constructor.
     */
    BeliefNode* createOrGetChild(Action const& action, Observation const& obs);

protected:
    /* -------------- Particle management / sampling ---------------- */
    /** Adds the given history entry to this belief node. */
    virtual void addParticle(HistoryEntry* newHistEntry);

    /** Removes the given history entry from this belief node. */
    virtual void removeParticle(HistoryEntry* histEntry);

    /* -------------------- Tree-related setters  ---------------------- */
    /** Sets the mapping for this node. */
    void setMapping(std::unique_ptr<ActionMapping> mapping);
    /** Sets the history-derived information for this node. */
    void setHistoricalData(std::unique_ptr<HistoricalData> data);

    /** The solver that this belief node belongs to. */
    Solver* solver_;

    /** The ID of this node. */
    long id_;
    /** The depth of this node in the tree. */
    long depth_;
    /** The observation entry that is this node's parent / owner. */
    ObservationMappingEntry* parentEntry_;

    /** The smart history-based data, to be used for history-based policies. */
    std::unique_ptr<HistoricalData> data_;
    /** The set of particles belonging to this node. */
    oppt::RandomAccessSet<HistoryEntry*> particles_;
    /** The number of sequences that start at this node. */
    long nStartingSequences_;

    /** A mapping of actions to action children for this node. */
    std::unique_ptr<ActionMapping> actionMap_;

    /** The cached values for this belief node. */
    std::unordered_map<BaseCachedValue const*, std::unique_ptr<BaseCachedValue>> cachedValues_;
    /** Calculates and caches the estimated value of this node. */
    CachedValue<FloatType>* valueEstimator_;
};
} /* namespace abt */

#endif /* SOLVER_BELIEFNODE_HPP_ */
