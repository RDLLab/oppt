/** @file gps_search.hpp
 *
 * Contains the necessary classes for a GPS search strategy; this is done via an
 * implementation of StepGenerator and StepGeneratorFactory;
 * the latter can then be wrapped inside a BasicSearchStrategy.
 */
#ifndef SOLVER_GPS_SEARCH_HPP_
#define SOLVER_GPS_SEARCH_HPP_

#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "solvers/ABT/solver/search/search_interface.hpp"

#include "solvers/ABT/solver/search/action-choosers/gps_choosers.hpp"

namespace abt {
/** A generator for steps that uses GPS to select actions.
 *
 * The action will be selected using GPS as long as the last action has been tried before; once
 * an action that has never been tried before is encountered, the search will terminate.
 */
class GpsStepGenerator : public StepGenerator {
public:
    /** Creates a new GpsStepGenerator associated with the given solver, using the given
     * options for GPS.
     */
	GpsStepGenerator(SearchStatus &status, Solver *solver, choosers::GpsChooserOptions options);
    virtual ~GpsStepGenerator() = default;
    _NO_COPY_OR_MOVE(GpsStepGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry, State const *state, HistoricalData const *data, Action *action=nullptr) override;

private:
    /** The model to use to generate next steps. */
    Model *model;
    /** Settings for the GPS search. */
    choosers::GpsChooserOptions options;

    /** True iff the last action selected hadn't been tried before. */
    bool choseUnvisitedAction;
};



/** A factory class for generating instances of GpsStepGenerator. */
class GpsStepGeneratorFactory: public StepGeneratorFactory {
public:
    /** Creates a new factory associated with the given solver, and with the given options. */
	GpsStepGeneratorFactory(Solver *solver, choosers::GpsChooserOptions options);
    virtual ~GpsStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(GpsStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status, HistoryEntry const *entry, State const *state, HistoricalData const *data) override;
private:
    /** The associated solver. */
    Solver *solver;
    /** GPS search options */
    choosers::GpsChooserOptions options;
};

} /* namespace abt */

#endif /* SOLVER_GPS_SEARCH_HPP_ */
