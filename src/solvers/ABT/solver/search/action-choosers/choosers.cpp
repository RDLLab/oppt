/** @file choosers.cpp
 *
 * Contains implementations of basic action selection functions.
 */
#include "choosers.hpp"

#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionMappingEntry.hpp"

using std::cout;
using std::endl;

namespace abt {
namespace choosers {
std::unique_ptr<Action> max_action(BeliefNode const *node) {    
    std::unique_ptr<Action> maxAction = nullptr;
    FloatType maxQValue = -std::numeric_limits<FloatType>::infinity();

    ActionMapping *mapping = node->getMapping();    
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        FloatType qValue = entry->getMeanQValue();
        if (qValue > maxQValue) {
            maxQValue = qValue;
            maxAction = entry->getAction();
        }
    }
    return std::move(maxAction);
}

std::unique_ptr<Action> robust_action(BeliefNode const *node) {
    std::unique_ptr<Action> robustAction = nullptr;
    long maxVisitCount = 0;
    FloatType robustQValue = -std::numeric_limits<FloatType>::infinity();

    ActionMapping *mapping = node->getMapping();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        FloatType visitCount = entry->getVisitCount();
        FloatType qValue = entry->getMeanQValue();
        if (visitCount > maxVisitCount) {
            maxVisitCount = visitCount;
            robustQValue = qValue;
            robustAction = entry->getAction();
        } else if (visitCount == maxVisitCount && qValue > robustQValue) {
            robustQValue = qValue;
            robustAction = entry->getAction();
        }
    }
    return std::move(robustAction);
}

std::unique_ptr<Action> ucb_action(BeliefNode const *node, FloatType explorationCoefficient) {
    std::unique_ptr<Action> ucbAction = nullptr;
    FloatType maxUcbValue = -std::numeric_limits<FloatType>::infinity();

    ActionMapping *mapping = node->getMapping();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        // Ignore illegal actions.
        if (!entry->isLegal()) {
            continue;
        }

        FloatType tmpValue = entry->getMeanQValue()
                + explorationCoefficient
                        * std::sqrt(
                                std::log(mapping->getTotalVisitCount()) / entry->getVisitCount());
        if (!std::isfinite(tmpValue)) {
            debug::show_message("ERROR: Infinite/NaN value!?");
            sleep(10);
        }
        if (maxUcbValue < tmpValue) {
            maxUcbValue = tmpValue;
            ucbAction = entry->getAction();
        }
    }
    return std::move(ucbAction);
}
} /* namespace choosers */
} /* namespace abt */
