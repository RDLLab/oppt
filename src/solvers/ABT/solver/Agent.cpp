/** @file Agent.cpp
 *
 * Contains the implementation of the Agent class.
 */
#include "Agent.hpp"

#include "BeliefNode.hpp"
#include "BeliefTree.hpp"
#include "Solver.hpp"

using std::cout;
using std::endl;

namespace abt {

Agent::Agent(Solver *solver) :
        solver_(solver),
        currentBelief_(solver_->getPolicy()->getRoot()) {
}

Solver *Agent::getSolver() const {
    return solver_;
}
std::unique_ptr<Action> Agent::getPreferredAction() const {
    return currentBelief_->getRecommendedAction();
}
BeliefNode *Agent::getCurrentBelief() const {
    return currentBelief_;
}

void Agent::setCurrentBelief(BeliefNode *belief) {
    currentBelief_ = belief;    
}
void Agent::updateBelief(Action const &action, Observation const &observation) {    
    currentBelief_ = currentBelief_->createOrGetChild(action, observation);    
}
} /* namespace abt */
