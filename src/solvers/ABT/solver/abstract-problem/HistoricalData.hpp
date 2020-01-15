/** @file HistoricalData.hpp
 *
 * Defines an abstract base class for storing history-based heuristic information within a belief
 * node.
 */
#ifndef SOLVER_HISTORICALDATA_HPP_
#define SOLVER_HISTORICALDATA_HPP_

#include <memory>

#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"

namespace abt {
/** An abstract base class for history-based heuristic info; each HistoricalData will be owned
 * by a single belief node.
 *
 * In order to function a HistoricalData must be able to generate a new, derived HistoricalData
 * instance for a child belief node, based on the action and observation taken to get there.
 */
class HistoricalData {
public:
    HistoricalData() = default;
    virtual ~HistoricalData() = default;

    /** Creates a copy of this HistoricalData instance. */
    virtual std::unique_ptr<HistoricalData> copy() const = 0;

    /** Generates a new child HistoricalData for a new belief node, based on the action taken
     * and observation received in going to that child node.
     */
    virtual std::unique_ptr<HistoricalData> createChild(Action const &action,
            Observation const &observation) const = 0;

    /** Prints out the contents of this HistoricalData [optional] */
    virtual void print(std::ostream &/*os*/) const {};
};

/** Uses the virtual print method to output HistoricalData to a stream. */
inline std::ostream &operator<<(std::ostream &os, HistoricalData const &data) {
    data.print(os);
    return os;
}
} /* namespace abt */

#endif /* SOLVER_HISTORICALDATA_HPP_ */
