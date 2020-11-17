/** @file ModelChange.hpp
 *
 * Defines an interface for storing changes to a model; this also defines a class
 * for a sequence of model changes, which is represented by a map from change times to
 * the associated changes at those times.
 */
#ifndef SOLVER_MODELCHANGE_HPP_
#define SOLVER_MODELCHANGE_HPP_

#include <map>
#include <memory>
#include <vector>

#include "oppt/global.hpp"

namespace abt {
/** An interface for model changes. There are no mandatory methods, because it is up to each
 * individual implementation of Model to determine how it will interact with model changes.
 */
class ModelChange {
public:
    ModelChange() = default;
    virtual ~ModelChange() = default;
    _NO_COPY_OR_MOVE (ModelChange);
};

/** Defines a change sequence as a map of times to vectors of model changes. */
typedef std::map<long, std::vector<std::unique_ptr<ModelChange>>> ChangeSequence;
} /* namespace abt */

#endif /* SOLVER_MODELCHANGES_HPP_ */

