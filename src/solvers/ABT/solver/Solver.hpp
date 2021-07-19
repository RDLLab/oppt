/** @file Solver.hpp
 *
 * Contains the Solver class, which is the core ABT solver class. A solver also owns a model,
 * which is its representation of the POMDP problem.
 */
#ifndef SOLVER_SOLVER_HPP_
#define SOLVER_SOLVER_HPP_

#include <map>
#include <memory>        // for unique_ptr
#include <set>                          // for set
#include <unordered_set>
#include <unordered_map>
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "oppt/global.hpp"                     // for RandomGenerator

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Model.hpp"                    // for Model, Model::StepResult
#include "abstract-problem/Observation.hpp"              // for Observation
#include "oppt/options/Options.hpp"              // for Options
#include "abstract-problem/State.hpp"

#include "changes/ChangeFlags.hpp"               // for ChangeFlags

/** A namespace to hold all of the various classes used by the solver - particularly the main
 * Solver class, of course.
 */


namespace solvers{
    class ABT;
    class ABTMultithreaded;
}

namespace abt
{
class ActionPool;
class BackpropagationStrategy;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationPool;
class SearchStrategy;
class Serializer;
class StateInfo;
class StatePool;

/** The core class of the ABT algorithm.
 *
 * * The core data structures for a solver, all of which are owned by the solver
 * (i.e. stored in unique_ptr), are:
 * - Model, which represents the POMDP model used to solve the problem.
 * - BeliefTree, which is used to represent the policy.
 * - StatePool, which keeps a collection of all of the states encountered so far; it can
 *      also keep a StateIndex for efficient indexing of the states, e.g. via a spatial index.
 *      This allows efficient lookup of which states are affected by a given change.
 * - Histories, which represents the collection of Histories that make up the belief tree. This
 *      is the key structure to allow incremental modification of the policy, since individual
 *      histories can be updated without updating the entire tree.
 *
 * The core functionality of the Solver lies in three methods:
 * - improvePolicy(), which incrementally improves the policy stored within the solver by
 *      generating new history sequences starting from a specific belief node. * 
 *      and the action and observation taken to get there.
 * - applyChanges(), which updates the histories to account for changes in the model, and also
 *      updates the tree with the effects of those changes so that the policy will remain valid.
 */
class Solver
{
public:
    friend class Serializer;
    friend class TextSerializer;
    friend class solvers::ABT;
    friend class solvers::ABTMultithreaded;

    /** Constructs a new solver, based on the given POMDP model. */
    Solver(std::unique_ptr<Model> model);
    virtual ~Solver();
    _NO_COPY_OR_MOVE(Solver);

    /* ------------------ Simple getters. ------------------- */
    /** Returns the policy. */
    BeliefTree* getPolicy() const;
    /** Returns the state pool. */
    StatePool* getStatePool() const;
    /** Returns the model. */
    Model* getModel() const;
    /** Returns the options used by the solver and model. */
    oppt::Options const* getOptions() const;
    /** Returns the action pool. */
    ActionPool* getActionPool() const;
    /** Returns the observation pool. */
    ObservationPool* getObservationPool() const;

    /** Returns the estimation strategy. */
    EstimationStrategy* getEstimationStrategy() const;

    /** Returns the recommendation strategy. */
    SelectRecommendedActionStrategy* getRecommendationStrategy() const;


    /** Returns the serializer for this solver. */
    Serializer* getSerializer() const;

    /* ------------------ Initialization methods ------------------- */
    /** Full initialization - resets all data structures. */
    virtual void initializeEmpty();

    /* ------------------- Policy mutators ------------------- */
    /** Improves the policy by generating the given number of histories from the given belief node.
     *
     * - startNode - the belief node to search from (nullptr => use the root and sample initial
     * states from the model)
     * - numberOfHistories is the number of histories to make (-1 => default, 0 => no limit),
     * - maximumDepth is the maximum depth allowed in the tree (-1 => default), relative to the
     * starting belief node.
     * - timeout is the maximum allowed time in milliseconds (-1 => default, 0 => no timeout)
     */
    long improvePolicy(BeliefNode* startNode = nullptr,
                       long numberOfHistories = -1, long maximumDepth = -1, FloatType timeout = -1);    

    /** Resets the tree, so that the given belief will be the new root. */
    void resetTree(BeliefNode* newRoot);

    /** Prunes all sibling nodes of the given node in the tree, i.e. all nodes that have the same
     * parent belief node, but take a different action and observation.
     */
    long pruneSiblings(BeliefNode* node);

    /** Prunes the subtree rooted at the given node, deleting all of the histories and all of
     * the nodes in this subtree.
     */
    long pruneSubtree(BeliefNode* root);

    /* ------------------- Change handling methods ------------------- */
    /** Returns the current root node for changes. */
    BeliefNode* getChangeRoot() const;
    /** Sets the root node for the changes. nullptr = all nodes. */
    void setChangeRoot(BeliefNode* changeRoot);
    /** Returns true iff the given node is affected by the current changes, which is expressed
     * via the change root, and a map storing info as to whether or not nodes have been
     * affected.
     */
    bool isAffected(BeliefNode const* node);
    /** Applies any model changes that have been marked within the state pool.
     *
     * Changes are only applied at belief nodes that are descended from the change root,
     * or at all belief nodes if the change root is nullptr.
     */
    void applyChanges();

    /* ------------------ Display methods  ------------------- */
    /** Shows a belief node in a nice, readable way. */
    void printBelief(BeliefNode* belief, std::ostream& os);

    /** Prints a compact representation of the entire tree. */
    void printTree(std::ostream& os);

    /* -------------- Management of deferred backpropagation. --------------- */
    /** Returns true iff there are any incomplete deferred backup operations. */
    bool isBackedUp() const;
    /** Completes any deferred backup operations. */
    void doBackup();

    /* -------------- Methods to update the q-values in the tree. --------------- */
    /** Updates the approximate q-values of actions in the belief tree based on this history
     * sequence.
     *  - Use sgn=-1 to do a negative backup
     *  - Use firstEntryId > 0 to backup only part of the seqeunce instead of all of it.
     *  - Use propagateQChanges = false to defer backpropagation and only use the immediate values
     *   when updating.
     *   This is useful for batched backups as it saves on the cost of recalculating the estimate
     *   of the value of the belief.
     */
    void updateSequence(HistorySequence* sequence, int sgn = +1, long firstEntryId = 0,
                        bool propagateQChanges = true);

    /** Performs a deferred update on the q-value for the parent belief and action of the
     * given belief.
     *
     * deltaTotalQ - change in heuristic value at this belief node.
     *
     * deltaNContinuations - change in number of visits to this node that
     * continue onwards (and hence can be estimated using the q-value
     * this node).
     */
    void updateEstimate(BeliefNode* node, FloatType deltaTotalQ, long deltaNContinuations);

    /**
     * Updates the values for taking the given action and receiving the given
     * observation from the given belief node.
     *
     * Q(b, a) will change, but the updating of the estimated value of the belief will be
     * deferred.
     *
     * deltaTotalQ - change in total reward due to immediate rewards
     * deltaNVisits - number of new visits (usually +1, 0, or -1)
     */
    void updateImmediate(BeliefNode* node, Action const& action, Observation const& observation,
                         FloatType deltaTotalQ, long deltaNVisits);

protected:
    /* ------------------ Initialization methods ------------------- */
    /** Partial pre-initialization - helper for full initialization,
     *    and for loading from a file.
     */
    virtual void initialize();

    /* ------------------ Episode sampling methods ------------------- */
    /** Returns a function that will sample states from the given node.
     * nullptr => sample initial states from the model.
     */
    std::function<StateInfo *()> getStateSampler(BeliefNode* node);

    /** Runs multiple searches from the given start node and start states.
     *
     * Returns the actual number of histories generated. */
    long multipleSearches(BeliefNode* startNode, std::function<StateInfo *()> sampler,
                          long maximumDepth, long maxNumSearches, FloatType endTime);
    /** Searches from the given start node with the given start state. */
    virtual void singleSearch(BeliefNode* startNode, StateInfo* startStateInfo, long maximumDepth, Action *action=nullptr);
    /** Continues a pre-existing history sequence from its endpoint. */
    virtual void continueSearch(HistorySequence* sequence, long maximumDepth, Action *action=nullptr);

    /* ------------------ Private deferred backup methods. ------------------- */
    /** Adds a new node that requires backing up. */
    void addNodeToBackup(BeliefNode* node);
    /** Removes a node from the deferred backup queue. */
    void removeNodeToBackup(BeliefNode* node);

    /* ------------------ Private data fields ------------------- */
    /** The POMDP model */
    std::unique_ptr<Model> model_;
    /** The configuration setings. */
    oppt::Options const* options_;

    /** The serializer to be used with this solver. */
    std::unique_ptr<Serializer> serializer_;

    /** The pool of actions (used to generate action mappings) */
    std::unique_ptr<ActionPool> actionPool_;
    /** The pool of observations (used to generate observation mappings) */
    std::unique_ptr<ObservationPool> observationPool_;

    /** The pool of states. */
    std::unique_ptr<StatePool> statePool_;
    /** The full collection of simulated histories. */
    std::unique_ptr<Histories> histories_;
    /** The tree that stores the policy */
    std::unique_ptr<BeliefTree> policy_;

    /** The history corrector. */
    std::unique_ptr<HistoryCorrector> historyCorrector_;
    /** The strategy to use when searching the tree. */
    std::unique_ptr<SearchStrategy> searchStrategy_;
    /** The strategy to use when searching the tree. */
    std::unique_ptr<SelectRecommendedActionStrategy> recommendationStrategy_;

    /** The strategy for estimating the value of a belief node based on actions from it. */
    std::unique_ptr<EstimationStrategy> estimationStrategy_;

    /** The nodes to be updated, sorted by depth (deepest first) */
    std::map<int, std::set<BeliefNode*>, std::greater<int>> nodesToBackup_;

    /** The root node for changes that will be applied. */
    BeliefNode* changeRoot_;

    /** A map to store which nodes are affected by changes and which are not. */
    std::unordered_map<BeliefNode const*, bool> isAffectedMap_;
};
} /* namespace abt */

#endif /* SOLVER_SOLVER_HPP_ */
