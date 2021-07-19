/** @file enumerated_actions.hpp
 *
 * Provides a default implementation for the action mapping interfaces in terms of an enumerated
 * set of actions. This is just like the discretized action mapping, but there is only one action
 * in each bin.
 *
 * Indeed, the actual mapping classes are the same as those for discretized actions;
 * the enumerated action case is handled simply by providing implementations for the pure virtual
 * methods of DiscretizedActionPool.
 */
#ifndef SOLVER_ENUMERATED_ACTIONS_HPP_
#define SOLVER_ENUMERATED_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "oppt/global.hpp"
#include "solvers/ABT/solver/RandomAccessSet.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"
#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/DiscretizedPoint.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionPool.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/mappings/actions/discretized_actions.hpp"

namespace abt {
class ActionPool;

/** Defines an action pool that uses an enumerated set of actions.
 *
 * This class keeps a vector of all of the possible actions, which it then uses in order to
 * conveniently implement the abstract class DiscretizedActionPool.
 */
class EnumeratedActionPool : public abt::DiscretizedActionPool {
  public:
    /** Creates a new EnumeratedActionPool from the given model, with the given vector of actions
     * as the enumerated vector of all of the actions, in order.
     */
    EnumeratedActionPool(Model *model, std::vector<std::unique_ptr<DiscretizedPoint>> allActions);
    virtual ~EnumeratedActionPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedActionPool);

    virtual long getNumberOfBins() override;
    virtual std::unique_ptr<Action> sampleAnAction(long binNumber) override;
    /** This provides a default implementation of createBinSequence(), which works by simply
     * creating a vector of all the bins [0, 1, ..., getNumberOfBins()-1], and shuffling that
     * vector.
     *
     * This means that the actions will be taken in a random order, but the order is determined
     * at the time this mapping is created. Taking this approach means the same interface can be
     * used to make a deterministic, prioritized list of actions instead..
     */
    virtual std::vector<long> createBinSequence(BeliefNode *node) override;

  private:
    /** The random number engine, taken from the model. */
    RandomGenerator *randGen_;
    /** The vector of all of the possible actions. */
    std::vector<std::unique_ptr<DiscretizedPoint>> allActions_;
};

/** Since we're just using DiscretizedActionMap to do all of the work, we can simply re-use
 * DiscretizedActionTextSerializer to do the work of serializing enumerated mappigns.
 */
typedef DiscretizedActionTextSerializer EnumeratedActionTextSerializer;
} /* namespace abt */

#endif /* SOLVER_ENUMERATED_ACTIONS_HPP_ */
