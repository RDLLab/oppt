/** @file estimators.cpp
 *
 * Provides implementations of some core estimation strategies.
 */
#include "estimators.hpp"

#include "solvers/ABT/solver/cached_values.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionMappingEntry.hpp"

namespace abt {
namespace estimators {
double average(BeliefNode const *node) {
    ActionMapping *mapping = node->getMapping();
    if (mapping->getNumberOfVisitedEntries() == 0) {
        return 0;
    }

    double totalQValue = 0;
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        totalQValue += entry->getTotalQValue();
    }

    double averageQValue = totalQValue / mapping->getTotalVisitCount();
    if (!std::isfinite(averageQValue)) {
        debug::show_message("Non-finite Q!");
    }
    return averageQValue;
}

double max(BeliefNode const *node) {
    ActionMapping *mapping = node->getMapping();
    if (mapping->getNumberOfVisitedEntries() == 0) {
        return 0;
    }

    double maxQValue = -std::numeric_limits<double>::infinity();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double qValue = entry->getMeanQValue();
        if (qValue > maxQValue) {
            maxQValue = qValue;
        }
    }
    return maxQValue;
}

double robust(BeliefNode const *node) {
    ActionMapping *mapping = node->getMapping();
    if (mapping->getNumberOfVisitedEntries() == 0) {
        return 0;
    }

    long maxVisitCount = 0;
    double robustQValue = -std::numeric_limits<double>::infinity();
    for (ActionMappingEntry const *entry : mapping->getVisitedEntries()) {
        double visitCount = entry->getVisitCount();
        double qValue = entry->getMeanQValue();
        if (visitCount > maxVisitCount) {
            maxVisitCount = visitCount;
            robustQValue = qValue;
        } else if (visitCount == maxVisitCount && qValue > robustQValue) {
            robustQValue = qValue;
        }
    }
    return robustQValue;
}
} /* namespace estimators */

EstimationFunction::EstimationFunction(std::function<double(BeliefNode const *)> function) :
            function_(function) {
}

void EstimationFunction::setValueEstimator(Solver */*solver*/, BeliefNode *node) {
    std::unique_ptr<CachedValue<double>> cachedValue = std::make_unique<CachedValue<double>>(node,
            function_);
    node->setValueEstimator(cachedValue.get());
    node->addCachedValue(std::move(cachedValue));
}
} /* namespace abt */
