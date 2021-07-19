/** @file discretized_actions.hpp
 *
 * Provides a default implementation for an action mapping that uses a set of discretized actions,
 * i.e. there is an enumerated set of action categories, and the actions in each of these
 * categories will map to the same child nodes in the belief tree. In order to use this mapping,
 *
 *
 * Since the number of bins is pre-set and finite, this mapping class simply stores the action
 * mapping entries for each belief node in an array, where the index into the array is simply the
 * number of the bin.
 *
 * This involves subclasses of the following abstract classes:
 * -ActionPool
 * -ActionMapping
 * -ActionMappingEntry
 *
 * as well as a serialization class providing methods to serialize this particular kind of
 * action pool and action mapping.
 */
#ifndef SOLVER_DISCRETIZED_ACTIONS_HPP_
#define SOLVER_DISCRETIZED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "oppt/global.hpp"
#include "solvers/ABT/solver/LinkedHashSet.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"
#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/ActionNode.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionPool.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"

namespace abt {
class DiscretizedActionMapEntry;

/** An abstract implementation of the ActionPool interface that considers actions in terms of
 * discrete bins.
 *
 * This implementation does not distinguish between the actions inside any given bin; however,
 * it allows a custom method to be given to sample an action from a single bin. This allows
 * different actions within the same bin to be sampled, although they will not be considered as
 * different from the solver's perspective.
 *
 * A concrete implementation of this abstract class requires implementations for
 * getNumberOfBins() and sampleAnAction() in order to define the discrete bins; each individual
 * action must also implement the DiscretizedPoint interface so that the bin number for a
 * specific action can be determined.
 *
 * Additionally, the createBinSequence() method must be implemented so that the initial set of
 * actions to try, and the order to try them in, will be set.
 */
class DiscretizedActionPool: public abt::ActionPool {
    friend class DiscretizedActionMap;
  public:
    DiscretizedActionPool() = default;
    virtual ~DiscretizedActionPool() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionPool);

    /** Returns the number of bins in the action discretization; the bins are enumerated
     * 0, 1, ... getNumberOfBins()-1
     */
    virtual long getNumberOfBins() = 0;

    /** Samples an action from bin with the given number. */
    virtual std::unique_ptr<Action> sampleAnAction(long binNumber) = 0;

    /** Creates an initial sequence of bins to try for the given belief node. This has two effects:
     * - The getNextActionToTry() method will return these actions in the given order
     * - Any actions *not* included will be marked as "illegal", and will be completely ignored
     *      by UCB unless they are later marked as legal.
     */
    virtual std::vector<long> createBinSequence(BeliefNode *node) = 0;

    virtual std::unique_ptr<ActionMapping> createActionMapping(BeliefNode *node) override;

  private:
};

/** A concrete class implementing ActionMapping for a discretized action space.
 *
 * This class stores its mapping entries in an array whose size is determined at runtime based on
 * the # of bins in the action pool.
 */
class DiscretizedActionMap: public abt::ActionMapping {
  public:
    friend class DiscretizedActionTextSerializer;
    friend class DiscretizedActionMapEntry;

    /** Constructs a new DiscretizedActionMap, which will be owned by the given belief node,
     * and be associated with the given DiscretizedActionpool.
     *
     * The given bin sequence defines which actions will be tried and in which order - any actions
     * not in the sequence will be considered illegal and should be ignored by UCB.
     */
    DiscretizedActionMap(BeliefNode *owner, DiscretizedActionPool *pool,
            std::vector<long> binSequence);

    // Default destructor; copying and moving disallowed!
    virtual ~DiscretizedActionMap();
    _NO_COPY_OR_MOVE(DiscretizedActionMap);

    /* -------------- Creation and retrieval of nodes. ---------------- */
    virtual ActionNode *getActionNode(Action const &action) const override;
    virtual ActionNode *createActionNode(Action const &action) override;
    virtual long getNChildren() const override;

    virtual void deleteChild(ActionMappingEntry const *entry) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const override;

    virtual long getNumberOfVisitedEntries() const override;
    virtual std::vector<ActionMappingEntry const *> getVisitedEntries() const override;
    virtual ActionMappingEntry *getEntry(Action const &action) override;
    virtual ActionMappingEntry const *getEntry(Action const &action) const override;

    /* ----------------- Methods for unvisited actions ------------------- */
    /** Returns the next action to be tried for this node, or nullptr if there are no more. */
    virtual std::unique_ptr<Action> getNextActionToTry() override;

    /* -------------- Retrieval of general statistics. ---------------- */
    virtual long getTotalVisitCount() const override;

  protected:
    /** The pool associated with this mapping. */
    DiscretizedActionPool *pool_;

    /** The number of bins in the mapping; equal to the number of entries in the array. */
    long numberOfBins_;
    /** The array of mapping entries - the bin number corresponds to the position in the array. */
    std::unique_ptr<DiscretizedActionMapEntry[]> entries_;
    /** The number of action node children that have been created. */
    long nChildren_;
    /** The number of entries with nonzero visit counts. */
    long numberOfVisitedEntries_;

    /** The sequence of bins to be visited. */
    oppt::LinkedHashSet<long> binSequence_;

    /** The total of the visit counts of all of the individual entries. */
    long totalVisitCount_;
};


/** A concrete class implementing ActionMappingEntry for a discretized action space.
 *
 * Each entry stores its bin number and a reference back to its parent map, as well as a child node,
 * visit count, total and mean Q-values, and a flag for whether or not the action is legal.
 */
class DiscretizedActionMapEntry : public abt::ActionMappingEntry {
    friend class DiscretizedActionMap;
    friend class DiscretizedActionTextSerializer;

  public:
    virtual ~DiscretizedActionMapEntry(){}
    virtual ActionMapping *getMapping() const override;
    virtual std::unique_ptr<Action> getAction() const override;
    virtual ActionNode *getActionNode() const override;
    virtual long getVisitCount() const override;
    virtual FloatType getTotalQValue() const override;
    virtual FloatType getMeanQValue() const override;
    virtual bool isLegal() const override;

    /** Returns the bin number associated with this entry. */
    long getBinNumber() const;

    virtual bool update(long deltaNVisits, FloatType deltaTotalQ) override;
    virtual void setLegal(bool legal) override;

  protected:
    /** The bin number of this entry. */
    long binNumber_ = -1;
    /** The parent action mapping. */
    DiscretizedActionMap *map_ = nullptr;
    /** The child action node, if one exists. */
    std::unique_ptr<ActionNode> childNode_ = nullptr;
    /** The visit count for this edge. */
    long visitCount_ = 0;
    /** The total Q-value for this edge. */
    FloatType totalQValue_ = 0;
    /** The mean Q-value for this edge => should be equal to totalQValue_ / visitCount_ */
    FloatType meanQValue_ = 0;
    /** True iff this edge is legal. */
    bool isLegal_ = false; // Entries are illegal by default.
};

/** A partial implementation of the Serializer interface which provides serialization methods for
 * the above discretized action mapping classes.
 */
class DiscretizedActionTextSerializer: virtual public abt::Serializer {
  public:
    DiscretizedActionTextSerializer() = default;
    virtual ~DiscretizedActionTextSerializer() = default;
    _NO_COPY_OR_MOVE(DiscretizedActionTextSerializer);

    virtual void saveActionPool(
            ActionPool const &actionPool, std::ostream &os) override;
    virtual std::unique_ptr<ActionPool> loadActionPool(
            std::istream &is) override;
    virtual void saveActionMapping(ActionMapping const &map,
            std::ostream &os) override;
    virtual std::unique_ptr<ActionMapping> loadActionMapping(BeliefNode *node,
            std::istream &is) override;

    /** Loads the data from the input stream into the given DiscretizedActionMap. */
    virtual void loadActionMapping(DiscretizedActionMap &discMap, std::istream &is);
};
} /* namespace abt */

#endif /* SOLVER_DISCRETIZED_ACTIONS_HPP_ */
