/** @file gps_choosers.hpp
 *
 * Defines some useful basic functions for selecting an action from a belief node
 * for the GPS search functionality.
 */
#ifndef SOLVER_GPS_CHOOSERS_HPP_
#define SOLVER_GPS_CHOOSERS_HPP_

#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

#include "solvers/ABT/solver/abstract-problem/Model.hpp"

namespace abt {
class BeliefNode;
class Solver;

namespace choosers {

struct GpsMaxRecommendationOptions {

	/** This defines the type of gps search used. I.e. the stencil, etc. */
	enum{GOLDEN=0, COMPASS=1} searchType = GOLDEN;

	/** Sets the number of dimensions the gps search ought to search over. */
	size_t dimensions = 1;

	/** This is the maximum number of dimensions gps search expects at compile time.
	 *
	 * Gps search uses template code which must be instanciated at compile time. This
	 * value determines up to which dimensionality the code is generated.
	 *
	 * Setting this value high will slow down compilation and bloat the final executable.
	 */
	static const size_t maxDimensions = 12;

	/** Allows to quickly disable GPS search in this case only the fixed actions are considered. */
	bool disableGpsSearch = false;

	enum{MEAN=0, ROBUST=1} recommendationMode = MEAN;
};


struct GpsChooserOptions {

	/** This defines the type of gps search used. I.e. the stencil, etc. */
	enum{GOLDEN=0, COMPASS=1} searchType = GOLDEN;

	/** Sets the number of dimensions the gps search ought to search over. */
	size_t dimensions = 1;

	/** This is the maximum number of dimensions gps search expects at compile time.
	 *
	 * Gps search uses template code which must be instanciated at compile time. This
	 * value determines up to which dimensionality the code is generated.
	 *
	 * Setting this value high will slow down compilation and bloat the final executable.
	 */
	static const size_t maxDimensions = 12;

	/** Allows to quickly disable GPS search in this case only the fixed actions are considered. */
	bool disableGpsSearch = false;

	/** The initial radius of the compass rose. A value of 1 means to max out the limits. It must be larger than zero. */
	FloatType initialCompassRadiusRatio = 1;

	/** The UCB exploration coefficient. */
	FloatType explorationCoefficient = 2;

	/** The GPS new search point coefficient */
	FloatType newSearchPointCoefficient = 2;

	/** The minimum number of visits each point of the current stencil ought to have before a child is created. */
	size_t minimumVisitsBeforeChildCreation = 1;

	/** The minimum distance the points need to have in order to create a child.
	 *
	 * The details are dependent on the GPS type.
	 */
	FloatType minimumChildCreationDistance = 0.1;
};


struct GpsChooserResponse {
	std::unique_ptr<Action> action = nullptr;
	bool actionIsVisited;
	GpsChooserResponse(): actionIsVisited(false) {}
	GpsChooserResponse(std::unique_ptr<Action>&& theAction, const bool theActionIsVisited): action(std::move(theAction)), actionIsVisited(theActionIsVisited) {}
};

/** Returns the action with the maximum estimate Q-value. */
GpsChooserResponse gps_max_action(BeliefNode const *node, const GpsMaxRecommendationOptions& options);

/** Returns the action with the highest visit count (ties are broken by max. value)
 *
 * It is not implemented for now.
 *
 * */
//GpsChooserResponse gps_robust_action(BeliefNode const *node, const GpsChooserOptions& options);

/** Returns the action with the highest UCB value and performs the GPS tree search. */
GpsChooserResponse gps_ucb_action(BeliefNode const *node, const Model& model, const GpsChooserOptions& options);

} /* namespace choosers */
} /* namespace abt */

#endif /* SOLVER_GPS_CHOOSERS_HPP_ */
