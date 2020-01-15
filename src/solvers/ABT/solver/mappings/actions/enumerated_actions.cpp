/** @file enumerated_actions.cpp
 *
 * Contains an implementation for the action mapping interfaces in terms of an enumerated
 * set of actions, which is done via a simple implementation of the abstract class
 * DiscretizedActionPool.
 */
#include "enumerated_actions.hpp"

namespace abt {
/* ------------------- EnumeratedActionPool ------------------- */
EnumeratedActionPool::EnumeratedActionPool(Model *model,
        std::vector<std::unique_ptr<DiscretizedPoint>> allActions) :
        randGen_(model->getRandomGenerator()),
        allActions_(std::move(allActions)) {
}
long EnumeratedActionPool::getNumberOfBins() {
    return allActions_.size();
}
std::unique_ptr<Action> EnumeratedActionPool::sampleAnAction(
        long binNumber) {
    return allActions_[binNumber]->copy();
}

std::vector<long> EnumeratedActionPool::createBinSequence(BeliefNode */*node*/) {
    std::vector<long> bins;
    for (int i = 0; i < getNumberOfBins(); i++) {
        bins.push_back(i);
    }
    std::shuffle(bins.begin(), bins.end(), *randGen_);
    return std::move(bins);
}
} /* namespace abt */



