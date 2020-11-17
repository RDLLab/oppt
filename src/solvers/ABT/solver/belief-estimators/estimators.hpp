/** @file estimators.hpp
 *
 * Defines an interface for how backpropagation will be handled; this is done by providing a custom
 * way to estimate the value of a belief based on the q-values of its actions.
 *
 * This file also provides a few basic implementations of this interface via three core functions;
 * these allow for an estimate of the value of the node via one of:
 * average - the visit-weighted average of its action children
 * max - the maximum value of its action children
 * robust - the value of the action child with the greatest number of visits
 */
#ifndef SOLVER_ESTIMATORS_HPP_
#define SOLVER_ESTIMATORS_HPP_

#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

namespace abt {
class BeliefNode;
class Solver;

/** An abstract base class to control how the values of belief nodes will be estimated.
 *
 * Every time a new belief node is created, the estimation strategy will be queried to set up
 * the estimator
 */
class EstimationStrategy {
public:
    EstimationStrategy() = default;
    virtual ~EstimationStrategy() = default;
    /** Sets the value estimator for the given belief node. */
    virtual void setValueEstimator(Solver *solver, BeliefNode *node) = 0;
};

/** A simple implementation of EstimationStrategy using a functional programming interface. */
class EstimationFunction : public EstimationStrategy {
public:
    /** Constructs a new estimator based on a function that takes a BeliefNode and returns a
     * FloatType.
     */
    EstimationFunction(std::function<FloatType(BeliefNode const *)> function);

    // Implementation of the key virtual setter method.
    virtual void setValueEstimator(Solver *solver, BeliefNode *node);

private:
    /** The function used by this estimation strategy .*/
    std::function<FloatType(BeliefNode const *)> function_;
};

//
namespace estimators {
/** Returns the average of the q-values of the actions that can be taken from this node, weighted
 * by the number of visits for each action. */
FloatType average(BeliefNode const *node);
/** Returns the maximum q-value among the actions that can be taken from this node. */
FloatType max(BeliefNode const *node);
/** Returns the q-value of the action taken most frequently from this node. */
FloatType robust(BeliefNode const *node);
} /* namespace estimators */
} /* namespace abt */

#endif /* SOLVER_ESTIMATORS_HPP_ */
