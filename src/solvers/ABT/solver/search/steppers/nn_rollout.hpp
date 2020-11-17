/** @file nn_rollout.hpp
 *
 * Contains the necessary classes for an approximate-nearest-neighbor-based rollout strategy;
 * this is done via an implementation of StepGenerator and StepGeneratorFactory;
 * the latter can then be wrapped inside a BasicSearchStrategy.
 */
#ifndef SOLVER_NN_ROLLOUT_HPP_
#define SOLVER_NN_ROLLOUT_HPP_

#include <unordered_map>

#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "solvers/ABT/solver/search/search_interface.hpp"

namespace abt {
/** The data associated with a near neighbor. */
struct NnData {
    /** The closest neighbor found so far for this node. */
    BeliefNode *neighbor = nullptr;
};

/** A factory class for creating individual instances of the NN-based rollout strategy.
 *
 * This class also keeps track of a mapping of nodes to near neighbors for those nodes, which
 * can then be used by the individual NNRolloutGenerator instances.
 */
class NnRolloutFactory: public StepGeneratorFactory {
public:
    /** Creates a new NnRolloutFactory associated with the given solver, and with the given
     * max # of NN comparisons to do, and the given maximum distance to be considered a "near"
     * neighbor.
     */
    NnRolloutFactory(Solver *solver, long maxNnComparisons, FloatType maxNnDistance);
    virtual ~NnRolloutFactory() = default;
    _NO_COPY_OR_MOVE(NnRolloutFactory);

    /** Finds an approximate nearest neighbor for the given belief node. */
    virtual BeliefNode* findNeighbor(BeliefNode *beliefNode);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data) override;

private:
    /** The associated solver. */
    Solver *solver_;
    /** The maximum number of NN comparisons to try for a new rollout. */
    long maxNnComparisons_;
    /** The maximum allowable distance to be considered a near neighbor. */
    FloatType maxNnDistance_;
    /** A mapping from belief nodes to data about their near neighbors .*/
    std::unordered_map<BeliefNode *, NnData> nnMap_;
};

/** Implementation of an NN-based StepGenerator instance. */
class NnRolloutGenerator: public StepGenerator {
public:
    /** Creates a new NN-based StepGenerator with the given solver, and with the given neighbor
     * belief node as its starting neighbor node.
     *
     * From there, the NN Rollout works by following the policy that is represented in the belief
     * tree, but starting from the neighbor node rather than the actual current belief node.
     */
    NnRolloutGenerator(SearchStatus &status, Solver *solver, BeliefNode *neighbor);
    virtual ~NnRolloutGenerator() = default;
    _NO_COPY_OR_MOVE(NnRolloutGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data, Action *action=nullptr) override;

private:
    /** The model of the associated solver; used to generate steps. */
    Model *model_;
    /** The current neighbor node - this will be a descendant of the initial neighbor. */
    BeliefNode *currentNeighborNode_;
};

} /* namespace abt */

#endif /* SOLVER_NN_ROLLOUT_HPP_ */
