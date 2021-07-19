/** @file gps_choosers.cpp
 *
 * Contains implementations of action selection functions for GPS search.
 */
#include "gps_choosers.hpp"

#include <utility>

#include "choosers.hpp"

#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/mappings/actions/continuous_actions.hpp"

namespace abt {
namespace choosers {

namespace gps_detail {



/** The hierarchy tree used for gps search */
template<class PARENT, size_t DIMENSIONS, size_t ACTION_SIZE, size_t CHILDREN_SIZE>
class GpsHierarchyDataBase: public abt::ChooserDataBase<PARENT> {
	typedef GpsHierarchyDataBase This;
	typedef PARENT Parent;
public:
	typedef ContinuousActionMapEntry ActionEntry;

	/** The number of dimensions this search has */
	static const size_t dimensions = DIMENSIONS;

	/** The number of actions this stencil has */
	static const size_t actionSize = ACTION_SIZE;

	/** the number of children this stencil has.
	 *
	 * Note: usually actionSize==childrenSize. While other patterns are possible in theory,
	 * the search algorithm does currently not accommodate for that.
	 */
	static const size_t childrenSize = CHILDREN_SIZE;

protected:
	/** create the hierarchy data. */
	GpsHierarchyDataBase(): lower(), upper(), actions(), children() {
		lower.fill(0);
		upper.fill(0);
		actions.fill(nullptr);
		//children.fill(nullptr); // this is a uniq_ptr, that gets initialised with null anyway.
	};
	virtual ~GpsHierarchyDataBase() = default;
	_NO_COPY_OR_MOVE(GpsHierarchyDataBase);


public:

	/** the lower values of the search rectangle. */
	std::array<FloatType, dimensions> lower;

	/** the upper values of the search rectangle. */
	std::array<FloatType, dimensions> upper;

	/** the action map entries we are dealing with here. */
	std::array<ActionEntry*, actionSize> actions;

	/** our children (if any). */
	std::array<std::unique_ptr<Parent>, childrenSize> children;

	/** Return a marker if the entry is on the active path (highest Q-value).
	 *
	 * This implementation assumes that actions and children of the same index belong together.
	 * If another pattern is used, this must be overridden in the Parent.
	 */
	virtual std::string getActionMarker(const ActionEntry* entry) const override {

		if (actionSize!=childrenSize) {
			debug::show_message("Warning: this version of getActionMarker only works if actions and children of the same index belong together. This is not the case.");
			return "";
		}

		std::string result;

		size_t level=0;
		This const* current = this;
		while(current != nullptr) {

			size_t bestIndex = 0;
			FloatType bestQValue = -std::numeric_limits<FloatType>::infinity();

			for (size_t i=0; i<current->actions.size(); i++){
				if ( (current->actions[i]!=nullptr) && (current->actions[i]->getMeanQValue() > bestQValue) ){
					bestQValue = current->actions[i]->getMeanQValue();
					bestIndex = i;
				}
			}

			if (current->actions[bestIndex] == nullptr) break;

			for (size_t i=0; i<current->actions.size(); i++){
				if ( current->actions[i]== entry){
					if (i==bestIndex) {
						result += 'A' + char(level);
					} else {
						result += 'a' + char(level);
					}
				}
			}

			current = current->children[bestIndex].get();
			level++;
		}

		return result;

	}

	/** a method to load the hierarchy data from a stream
	 *
	 * An implementation of this class must provide "loadFromString(std::string)" in order to populate its fields.
	 *
	 */
	static std::unique_ptr<Parent> loadFromStream(std::istream& is, ContinuousActionMap& map) {
		std::unique_ptr<Parent> result = std::make_unique<Parent>();
		result->loadFromStream_impl(is, map);
		return std::move(result);
	}

protected:
	/** method to save the data to a stream
	 *
	 * An implementation of this class must provide "std::string saveToString()" in order to save its fields.
	 * The returned string may only be one line (no line breaks) and may not contain the character that is
	 * found in Parent::serialisationTerminator;
	 *
	 */
	virtual void saveToStream_real(std::ostream& os, const ContinuousActionMap& /*map*/) const override {
		saveToStream_impl(os,"","");
	}



private:

	/** a method that does the work for saveToStream and recurses through the tree. */
	void saveToStream_impl(std::ostream& os, const std::string& ownPrefix, const std::string& prefix) const {

		// draw some nice lines for humans. When parsing we delete everything up to the first "<".
		os << ownPrefix << "+--";
		for (int i=0; i< 40-int(ownPrefix.size()); i++) {
			os << "-";
		}
		os << "<";

		//save the parent data first
		os << static_cast<const Parent*>(this)->saveToString() << Parent::serialisationTerminator << " ";

		// save the lower bounds
		os << "lower: ";
		for (FloatType value : lower) {
			os << value << " ";
		}

		// save the upper bounds
		os << "upper: ";
		for (FloatType value : upper) {
			os << value << " ";
		}

		// mark which actions exist
		os << "actions: ";
		for (auto action : actions) {
			os << (action != nullptr) << " ";
		}

		// mark which children exist
		os << "children: ";
		size_t lastExistingChild = 0;
		for (size_t i=0; i< children.size(); i++) {
			os << (children[i] != nullptr) << " ";
			if (children[i] != nullptr) lastExistingChild = i;
		}

		// some informational things that are not used for loading but handy for debugging
		os << " [( qMeans, visits ): ";
		for (ActionEntry* action : actions) {
			if (action != nullptr) {
				os << "( "<< action->getMeanQValue() << ", " << action->getVisitCount() << " ) ";
			} else {
				os << "n/a ";
			}
		}
		os << "]";

		// finally end the line.
		os << std::endl;

		// the children. when loading we can use the values in the "children" field above to check for existence of children.
		for (size_t i = 0; i < children.size(); i++) {
			if (children[i] != nullptr) {
				if (i+1 != children.size()) {
					children[i]->saveToStream_impl(os, prefix + "|-", prefix + (i>=lastExistingChild ? "  " : "| ") );
				} else {
					children[i]->saveToStream_impl(os, prefix + "`-", prefix + "  ");

				}
			}
		}


	}

	/** a method that does the work for loadFromStream and recurses through the tree. */
	void loadFromStream_impl(std::istream& is, ContinuousActionMap& map) {
		std::string line;
		std::getline(is, line);

		// get our bearings in the string
		size_t parentStart = line.find('<') + 1;
		size_t parentEnd = line.find(Parent::serialisationTerminator, parentStart);
		size_t start = parentEnd + 2;

		std::string dummy;
	    std::istringstream ss(line.substr(start));

		// we load ourself first in case the parent expects the bounds to be set already, etc.

		// load the lower bounds
		ss >> dummy;
		for (FloatType& value : lower) {
			ss >> value;
		}

		// load the upper bounds
		ss >> dummy;
		for (FloatType& value : upper) {
			ss >> value;
		}

		// load the action markers
		std::array<bool, actionSize> actionsExist;
		ss >> dummy;
		for (bool& marker : actionsExist) {
			ss >> marker;
		}


		// load the child markers
		std::array<bool, childrenSize> childrenExist;
		ss >> dummy;
		for (bool& marker : childrenExist) {
			ss >> marker;
		}

		// now load the parent
		static_cast<Parent*>(this)->loadFromString(line.substr(parentStart, parentEnd-parentStart));

		// now we have to find the actions
		for (size_t i = 0; i < actions.size(); i++) {
			if (actionsExist[i]) {
				actions[i] = map.getActionMapEntry( static_cast<Parent*>(this)->calculateActionCoordinates(i).data() );
			} else {
				actions[i] = nullptr;
			}
		}

		// now we load the children
		for (size_t i = 0; i < children.size(); i++) {
			if (childrenExist[i]) {
				children[i] = loadFromStream(is, map);
			} else {
				children[i] == nullptr;
			}
		}


	}


};



/** The hierarchy tree used for compass search */
template<size_t DIMENSIONS>
class CompassHierarchyData final: public GpsHierarchyDataBase<CompassHierarchyData<DIMENSIONS>, DIMENSIONS, 1+2*DIMENSIONS, 1+2*DIMENSIONS> {
	typedef CompassHierarchyData This;
	typedef GpsHierarchyDataBase<CompassHierarchyData<DIMENSIONS>, DIMENSIONS, 1+2*DIMENSIONS, 1+2*DIMENSIONS> Base;
public:
	using Base::dimensions;
	using Base::actionSize;
	using Base::lower;
	using Base::upper;
	using Base::actions;
	using Base::children;

	/** create the hierarchy data. */
	CompassHierarchyData(): point(), radius(0) {
		point.fill(0);
	};

	/** This method initialises the fields to those suitable for the root of the search tree. */
	void initialiseRoot(ContinuousActionMap& map, const GpsChooserOptions& options) {
		auto boundingBox = map.getActionPool()->getInitialBoundingBox(map.getOwner());

		if (boundingBox.size() != dimensions) {
			class InitialBoundingBoxHasWrongDimensions: public std::exception {
				virtual const char* what() const noexcept {
					return "The bounding box returned by the action pool has wrong dimensions. There is either a bug or a configuration error.";
				}
			} e;
			throw e;
		}

		radius = std::numeric_limits<FloatType>::infinity();

		for (size_t i=0; i<dimensions; i++) {
			lower[i] = boundingBox[i].first;
			upper[i] = boundingBox[i].second;
			point[i] = (boundingBox[i].first + boundingBox[i].second) / 2;

			FloatType maxRadius = (boundingBox[i].second - boundingBox[i].first) / 2;
			if (radius > maxRadius) {
				radius = maxRadius;
			}
		}

		radius *= options.initialCompassRadiusRatio;

		linkActions(map);

	}

	/** This function calculates the coordinates associated with the stencil.
	 *
	 * The values are used to initialise the actions field.
	 */
	std::array<FloatType,dimensions> calculateActionCoordinates(const size_t index) const {
		// we use the convention that index 0 is the center point. Then we go through the dimensions with the lower points. Then comes the higher points.
		std::array<FloatType, dimensions> result = point;
		if (index > 0) {
			if (index <= dimensions) {
				result[index-1] -= radius;
			} else {
				result[index-dimensions-1] += radius;

			}
		}
		return result;
	}

	void linkActions(ContinuousActionMap& map) {
		for (size_t i=0; i<actionSize; i++) {
			auto coordinates = calculateActionCoordinates(i);
			for (size_t dim=0; dim < dimensions; dim++) {
				if ( (coordinates[dim] < lower[dim]) || (coordinates[dim] > upper[dim]) ) {
					goto label_doNotCreateActionAndContinueWithNextAction; // use goto to break from this loop and continue with the outer one.
				}
			}
			actions[i] = map.createOrGetActionMapEntry(coordinates.data());

			label_doNotCreateActionAndContinueWithNextAction:;
		}
	}

	/** this function determines, whether another child is allowed to be created.
	 *
	 * The purpose is to evaluate the minimumChildCreationDistance option.
	 */
	bool mayCreateChild(const size_t index, const GpsChooserOptions& options) const {
		if (index==0) {
			return (radius > options.minimumChildCreationDistance);
		} else {
			return true;
		}
	}

	/** This function attempts to create the child with the correct index. */
	void createChild(const size_t index, ContinuousActionMap& map) {

		std::unique_ptr<This>& child = children[index];

		if (child != nullptr) return;

		child = std::make_unique<This>();

		child->lower = lower;
		child->upper = upper;

		child->point = calculateActionCoordinates(index);

		if (index == 0) {
			child->radius = radius / 2;
		} else {
			child->radius = radius;
		}

		child->linkActions(map);
	}

private:
	/** The current center point. */
	std::array<FloatType, dimensions> point;

	/** The radius if the compass rose */
	FloatType radius = 0;

public:
	// infrastructure for serialisation.

	static const char serialisationTerminator = '|';

	/** method to save the date to a stream */
	std::string saveToString() const {
		std::stringstream ss;
		ss << "point: ";
		for (FloatType val : point) {
			ss << val << " ";
		}
		ss << "radius: " << radius << " ";
		return ss.str();
	}

	/** method to save the date to a stream */
	void loadFromString(const std::string& line) {
		std::istringstream ss(line);
		std::string dummy;
		ss >> dummy;
		for (FloatType& val : point) {
			ss >> val;
		}
		ss >> dummy >> radius;
	}
};

/** The hierarchy tree used for golden section search */
class GoldenHierarchyData final: public GpsHierarchyDataBase<GoldenHierarchyData, 1, 2, 2> {
	typedef GoldenHierarchyData This;
	typedef GpsHierarchyDataBase<GoldenHierarchyData, 1, 2, 2> Base;

private:
	static constexpr const FloatType goldenRatio = 0.6180339887498948482045868343656381177203091798057628621354; // (sqrt(5)-1)/2;
	static constexpr const FloatType oneMinusGoldenRatio = 1 - goldenRatio;

public:
	using Base::dimensions;
	using Base::actionSize;
	using Base::lower;
	using Base::upper;
	using Base::actions;
	using Base::children;

	/** create the hierarchy data. */
	GoldenHierarchyData() = default;

	/** This method initialises the fields to those suitable for the root of the search tree. */
	void initialiseRoot(ContinuousActionMap& map, const GpsChooserOptions& /*options*/) {
		auto boundingBox = map.getActionPool()->getInitialBoundingBox(map.getOwner());

		if (boundingBox.size() != dimensions) {
			class InitialBoundingBoxHasWrongDimensions: public std::exception {
				virtual const char* what() const noexcept {
					return "The bounding box returned by the action pool has wrong dimensions. There is either a bug or a configuration error.";
				}
			} e;
			throw e;
		}

		for(size_t i=0; i< dimensions; i++) {
			lower[i] = boundingBox[i].first;
			upper[i] = boundingBox[i].second;
		}

		linkActions(map);

	}

	/** This function calculates the coordinates associated with the stencil.
	 *
	 * The values are used to initialise the actions field.
	 */
	std::array<FloatType,dimensions> calculateActionCoordinates(const size_t index) const {
		// we use the convention that index 0 is the lower point.
		if (index==0) {
			return { lower[0] * goldenRatio + upper[0] * oneMinusGoldenRatio };
		} else {
			return { lower[0] * oneMinusGoldenRatio + upper[0] * goldenRatio };
		}
	}

	void linkActions(ContinuousActionMap& map) {
		for (size_t i=0; i<actionSize; i++) {
			auto coordinates = calculateActionCoordinates(i);
			actions[i] = map.createOrGetActionMapEntry(coordinates.data());
		}
	}

	/** this function determines, whether another child is allowed to be created.
	 *
	 * The purpose is to evaluate the minimumChildCreationDistance option.
	 */
	bool mayCreateChild(const size_t /*index*/, const GpsChooserOptions& options) const {
		return (upper[0] - lower[0] > options.minimumChildCreationDistance);
	}

	/** This function attempts to create the child with the correct index. */
	void createChild(const size_t index, ContinuousActionMap& map) {

		std::unique_ptr<This>& child = children[index];

		if (child != nullptr) return;

		child = std::make_unique<This>();

		if (index==0) {
			child->lower = lower;
			child->upper = calculateActionCoordinates(1);
		} else {
			child->lower = calculateActionCoordinates(0);
			child->upper = upper;
		}

		child->linkActions(map);
	}

public:
	// infrastructure for serialisation.

	static const char serialisationTerminator = '|';

	/** method to save the data to a stream */
	std::string saveToString() const {
		// we don't have any extra data members.
		return "";
	}

	/** method to save the date to a stream */
	void loadFromString(const std::string& /*line*/) {
		// we don't have any extra data members.
		return;
	}
};


/** This class is a collection of static functions to perform the Gps search
 *
 * The main reason for making it a class is so the template parameter and typedefs
 * don't have to be repeated all the time. It is more like a templated namespace
 * than an actual class.
 *
 */
template<class GPS_STENCIL>
class GpsSearch {

	typedef GPS_STENCIL GpsStencil;
	typedef ContinuousActionMap ThisActionMap;
	typedef ContinuousActionMapEntry ThisActionMapEntry;
	typedef ContinuousActionConstructionDataBase ActionConstructionData;


	static void ensureChooserDataIsInitialised(ThisActionMap& map, const GpsChooserOptions& options) {
		if (map.chooserData == nullptr) {
			map.chooserData = std::make_unique<GpsStencil>();
			static_cast<GpsStencil&>(*map.chooserData).initialiseRoot(map, options);
		}
	}

	static FloatType calculateUcbScore(const ThisActionMapEntry& entry, const ThisActionMap& map, const GpsChooserOptions& options) {
		return entry.getMeanQValue() + sqrt(options.explorationCoefficient * std::log( map.getTotalVisitCount() ) / entry.getVisitCount()  );
	}

	static void gpsUcbAction_processFixed(BeliefNode const */*node*/, const Model& model, const GpsChooserOptions& options, const ThisActionMap& mapping, FloatType& bestPointScore, ThisActionMapEntry*& bestEntry ) {

		// Process the actions and calculate the ucb score.
		size_t unvisitedEntryCount = 0;
		for (ThisActionMapEntry* entry : mapping.getFixedEntries()) {

			if (entry->getVisitCount() == 0) {
				// count the unvisited entries.
				unvisitedEntryCount++;
			} else {

				// if we already have unvisited entries, the point score doesn't matter anyway, so don't bother calculating it.
				if (unvisitedEntryCount == 0) {

					FloatType pointScore = calculateUcbScore(*entry, mapping, options);
					if (pointScore >= bestPointScore) {
						bestPointScore = pointScore;
						bestEntry = entry;
					}

				}

			}
		}

		// If we have unvisited entries, return one of them at random.
		if (unvisitedEntryCount > 0) {

			// decide which unvisited entry to take.
			size_t selectEntryNumber;
			if (unvisitedEntryCount == 1) {
				selectEntryNumber = 0;
			} else {
				selectEntryNumber = model.getRandomGenerator()->operator()() % unvisitedEntryCount;
			}

			// find the entry again.
			size_t count = 0;
			for (ThisActionMapEntry* currentEntry : mapping.getFixedEntries()) {
				if (currentEntry->getVisitCount() == 0) {
					if (count==selectEntryNumber) {

						// done. We found our entry, return it to the caller.
						bestPointScore = std::numeric_limits<FloatType>::infinity();
						bestEntry = currentEntry;
						return;
					}
					count++;;
				}
			}

		}

	}

	static void gpsUcbAction_processGps(BeliefNode const */*node*/, const Model& model, const GpsChooserOptions& options, ThisActionMap& mapping, FloatType& bestPointScore, ThisActionMapEntry*& bestEntry ) {

		// we traverse the tree. If we decide to descend into a child, currentNode will be set to the child in the next round of the loop.
		GpsStencil* currentNode = static_cast<GpsStencil*>(mapping.chooserData.get());
		

		// when the loop is finished, currentNode will be null. Thus we keep a copy of it in previousNode so we know which leaf we reached in the end.
		GpsStencil* previousNode = nullptr;

		// We also want to know which child we want to add after the while loop.
		size_t bestEntryIndexAtPreviousNode = 0;

		// we need to know how far we descended into the tree.
		size_t hirarchyLevels = 0;

		// if we encounter actions that are not visited enough, we block child creation.
		bool blockCreationOfNewchild = false;


		while (currentNode != nullptr) {

			// keep track of our progress.
			hirarchyLevels++;
			previousNode = currentNode;


			// check if we have an unvisited entry. If more than one are unvisited, return one at random, otherwise the unvisited one.
			size_t unvisitedCount = 0;
			for (auto& action : currentNode->actions) {
				if ( (action != nullptr) && (action->getVisitCount() == 0) ) unvisitedCount++;
			}


			if (unvisitedCount > 0) {
				size_t selectEntryNumber;
				if (unvisitedCount == 1) {
					selectEntryNumber = 0;
				} else {
					selectEntryNumber = model.getRandomGenerator()->operator()() % unvisitedCount;
				}
				size_t count = 0;
				for (auto& entry : currentNode->actions) {
					if ( (entry != nullptr) && (entry->getVisitCount() == 0) ) {
						if (count == selectEntryNumber) {
							bestPointScore = std::numeric_limits<FloatType>::infinity();
							bestEntry = entry;							
							return;
						}
						count++;
					}
				}
			}


			// process the current actions.
			size_t bestEntryIndexAtCurrentNode = 0;
			FloatType bestEntryMeanAtCurrentNode = -std::numeric_limits<FloatType>::infinity();
			size_t index = 0;


			for (ThisActionMapEntry* entry : currentNode->actions) {

				FloatType pointScore;

				if (entry != nullptr) {
					if (entry->getVisitCount() < std::max(long(1), long(options.minimumVisitsBeforeChildCreation))) {
						blockCreationOfNewchild = true;
					}

					pointScore = calculateUcbScore(*entry, mapping, options);

					if (pointScore >= bestPointScore) {
						bestPointScore = pointScore;
						bestEntry = entry;
					}

					if (entry->getMeanQValue() >= bestEntryMeanAtCurrentNode) {
						bestEntryMeanAtCurrentNode = entry->getMeanQValue();
						bestEntryIndexAtCurrentNode = index;
					}
				}

				index++;
			}

			// save the best entry index for future reference
			bestEntryIndexAtPreviousNode = bestEntryIndexAtCurrentNode;

			// Process the next child.
			currentNode = currentNode->children[bestEntryIndexAtCurrentNode].get();

		} // end of while loop


		// We reached a leaf of the tree now. Determine whether we want to create a child.

		// Calculate how deep the hirarchy is allowed to be.
		FloatType desiredHirarchyLevels = 0;
		if (mapping.getTotalVisitCount() > 0) {
			desiredHirarchyLevels = log(mapping.getTotalVisitCount())/log(options.newSearchPointCoefficient);
		}


		// Create the child if we may.
		if (!blockCreationOfNewchild && (hirarchyLevels < desiredHirarchyLevels) && previousNode->mayCreateChild(bestEntryIndexAtPreviousNode, options) ) {

			previousNode->createChild(bestEntryIndexAtPreviousNode, mapping);


			// return an unvisited entry of the child at random.
			// If all are visited, we calculate the point scores and do normal UCP.
			currentNode = previousNode->children[bestEntryIndexAtPreviousNode].get();
			size_t unvisitedCount = 0;
			size_t nonNullCount = 0;
			for (auto& action : currentNode->actions) {
				if (action != nullptr) {
					nonNullCount++;
					if (action->getVisitCount() == 0) unvisitedCount++;
				}
			}

			if (unvisitedCount > 0) {
				// return an unvisited entry at random.
				size_t selectEntryNumber;
				if (unvisitedCount==1) {
					selectEntryNumber = 0;
				} else {
					selectEntryNumber = model.getRandomGenerator()->operator()() % unvisitedCount;
				}
				size_t count = 0;
				for (auto& entry : currentNode->actions) {
					if ( (entry != nullptr) && (entry->getVisitCount() == 0) ) {
						if (count == selectEntryNumber) {
							bestPointScore = std::numeric_limits<FloatType>::infinity();
							bestEntry = entry;							
							return;
						}
						count++;
					}
				}
			} else if (nonNullCount > 0){

				// calculate the point scores.
				for (auto& entry : currentNode->actions) {
					if (entry != nullptr) {
						FloatType pointScore = calculateUcbScore(*entry, mapping, options);
						if (pointScore >= bestPointScore) {
							bestPointScore = pointScore;
							bestEntry = entry;
						}
					}
				}

			} else {
				debug::show_message("This should not happen. We must have created a child with only NULL actions.");
			}


		}

	}


public:

	static GpsChooserResponse gpsUcbAction(BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {


		ThisActionMap& mapping = *(static_cast<ThisActionMap*>(node->getMapping()));


		// we traverse the tree to find the best node.
		FloatType bestPointScore = -std::numeric_limits<FloatType>::infinity();
		ThisActionMapEntry* bestEntry = nullptr;

		gpsUcbAction_processFixed(node, model, options, mapping, bestPointScore, bestEntry);

		// return if there is an unvisited fixed action.
		if ( (bestEntry!= nullptr) && (bestEntry->getVisitCount() == 0) ) {		        
			return GpsChooserResponse(bestEntry->getAction(), false);
		}

		// If Gps search is disabled, return what we have.
		if (options.disableGpsSearch) {
			if (bestEntry == nullptr) {
				class NoFixedActionsAndGpsSearchIsDisabled: public std::exception {
					virtual const char* what() const noexcept {
						return "Gps search is disabled and there are no fixed actions. This can't work. There is probably a wrong configuration option.";
					}
				} e;
				throw e;
			}			

			return GpsChooserResponse(bestEntry->getAction(), bestEntry->getVisitCount() > 0);
		}


		ensureChooserDataIsInitialised(mapping, options);

		gpsUcbAction_processGps(node, model, options, mapping, bestPointScore, bestEntry);


		if (bestEntry == nullptr) {
			class NoActionCouldBeSelected: public std::exception {
				virtual const char* what() const noexcept {
					return "Gps search did not return any usable action. Something is not right.";
				}
			} e;
			throw e;
		}		
		
		return GpsChooserResponse(bestEntry->getAction(), bestEntry->getVisitCount() > 0);

	}

	static GpsChooserResponse gpsMaxAction(BeliefNode const *node, const GpsMaxRecommendationOptions& options) {


		ThisActionMap& mapping = *(static_cast<ThisActionMap*>(node->getMapping()));


		// variables to store the best current action.
		FloatType bestPointScore = -std::numeric_limits<FloatType>::infinity();
		ThisActionMapEntry* bestEntry = nullptr;

		// we check the fixed actions.
		for (ThisActionMapEntry* entry : mapping.getFixedEntries()) {
			if (entry->getVisitCount() == 0) continue;

			FloatType pointScore;
			if (options.recommendationMode == GpsMaxRecommendationOptions::MEAN) {
				pointScore = entry->getMeanQValue();
			} else if (options.recommendationMode == GpsMaxRecommendationOptions::ROBUST){
				pointScore = entry->getVisitCount();
			} else {
				std::cout << "Unknown recommendation mode." << std::endl;
				return GpsChooserResponse(nullptr, false);
			}

			if (pointScore >= bestPointScore) {
				bestPointScore = pointScore;
				bestEntry = entry;
			}
		}

		if (!options.disableGpsSearch) {

			// we traverse the tree. If we decide to descend into a child, currentNode will be set to the child in the next round of the loop.
			GpsStencil* currentNode = static_cast<GpsStencil*>(mapping.chooserData.get());

			while (currentNode != nullptr) {

				// process the current actions.
				size_t bestCurrentEntryIndex = 0;
				FloatType bestCurrentEntryScore = -std::numeric_limits<FloatType>::infinity();

				// check all the actions.
				for (size_t index=0; index < currentNode->actions.size(); index++) {

					ThisActionMapEntry* entry = currentNode->actions[index];

					if ( (entry != nullptr) && (entry->getVisitCount() > 0)) {

						FloatType meanQValue = entry->getMeanQValue();

						FloatType pointScore;
						if (options.recommendationMode == GpsMaxRecommendationOptions::MEAN) {
							pointScore = meanQValue;
						} else if (options.recommendationMode == GpsMaxRecommendationOptions::ROBUST){
							pointScore = entry->getVisitCount();
						} else {
							std::cout << "Unknown recommendation mode." <<std::endl;
							return GpsChooserResponse(nullptr, false);
						}

						if (pointScore >= bestPointScore) {
							bestPointScore = pointScore;
							bestEntry = entry;
						}
						if (meanQValue >= bestCurrentEntryScore) {
							bestCurrentEntryScore = meanQValue;
							bestCurrentEntryIndex = index;
						}
					}
				}

				// process the child next.
				currentNode = currentNode->children[bestCurrentEntryIndex].get();
			}
		}

		if (bestEntry != nullptr) {
			return GpsChooserResponse(bestEntry->getAction(), bestEntry->getVisitCount() > 0);
		} else {
			debug::show_message("Warning: could not get a gps action. Falling back to considering all actions.");
			return GpsChooserResponse(choosers::max_action(node), true);
		}

	}


};


/* A routing function to call the appropriate template implementation based on run-time configuration options. */
template<size_t DIMENSIONS>
void route_gpsUcbAction_compass(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options);

template<>
void route_gpsUcbAction_compass<GpsChooserOptions::maxDimensions+1>(GpsChooserResponse& /*result*/, BeliefNode const* /*node*/, const Model& /*model*/, const GpsChooserOptions& /*options*/) {
	class TooManyDimensions: public std::exception {
		virtual const char* what() const noexcept {
			return "route_gpsUcbAction_compass: the dimensionality you requested wasn't instanciated during compile time. Please adjust GpsChooserOptions::maxDimensions.";
		}
	} e;
	throw e;
}

template<size_t DIMENSIONS>
void route_gpsUcbAction_compass(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {        
	if (DIMENSIONS==options.dimensions) {
		result = GpsSearch<CompassHierarchyData<DIMENSIONS>>::gpsUcbAction(node, model, options);
	} else {
		route_gpsUcbAction_compass<DIMENSIONS+1>(result, node, model, options);
	}
}


/* A routing function to call the appropriate template implementation based on run-time configuration options. */
void route_gpsUcbAction_golden(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
	if (options.dimensions==1) {
		result = GpsSearch<GoldenHierarchyData>::gpsUcbAction(node, model, options);
	} else {
		class TooManyDimensions: public std::exception {
			virtual const char* what() const noexcept {
				return "Golden section search only works in one dimension. Please change your options.";
			}
		} e;
		throw e;
	}
}

/* A routing function to call the appropriate template implementation based on run-time configuration options. */
void route_gpsUcbAction(GpsChooserResponse& result, BeliefNode const *node, const Model& model, const GpsChooserOptions& options) {
	if (options.searchType == GpsChooserOptions::COMPASS) {
		route_gpsUcbAction_compass<1>(result, node, model, options);
	} else if (options.searchType == GpsChooserOptions::GOLDEN) {
		route_gpsUcbAction_golden(result, node, model, options);
	} else {
		class UnknownSearchType: public std::exception {
			virtual const char* what() const noexcept {
				return "the GPS search type you requested is unknown. Please change your options.";
			}
		} e;
		throw e;
	}
}


/* A routing function to call the appropriate template implementation based on run-time configuration options. */
template<size_t DIMENSIONS>
void route_gpsMaxAction_compass(GpsChooserResponse& result, BeliefNode const *node, const GpsMaxRecommendationOptions& options);

template<>
void route_gpsMaxAction_compass<GpsChooserOptions::maxDimensions+1>(GpsChooserResponse& /*result*/, BeliefNode const* /*node*/, const GpsMaxRecommendationOptions& /*options*/) {
	class TooManyDimensions: public std::exception {
		virtual const char* what() const noexcept {
			return "route_gpsUcbAction_compass: the dimensionality you requested wasn't instantiated during compile time. Please adjust GpsChooserOptions::maxDimensions.";
		}
	} e;
	throw e;
}

template<size_t DIMENSIONS>
void route_gpsMaxAction_compass(GpsChooserResponse& result, BeliefNode const *node, const GpsMaxRecommendationOptions& options) {
	if (DIMENSIONS==options.dimensions) {
		result = GpsSearch<CompassHierarchyData<DIMENSIONS>>::gpsMaxAction(node, options);
	} else {
		route_gpsMaxAction_compass<DIMENSIONS+1>(result, node, options);
	}
}


/* A routing function to call the appropriate template implementation based on run-time configuration options. */
void route_gpsMaxAction_golden(GpsChooserResponse& result, BeliefNode const *node, const GpsMaxRecommendationOptions& options) {
	if (options.dimensions==1) {
		result = GpsSearch<GoldenHierarchyData>::gpsMaxAction(node, options);
	} else {
		class TooManyDimensions: public std::exception {
			virtual const char* what() const noexcept {
				return "Golden section search only works in one dimension. Please change your options.";
			}
		} e;
		throw e;
	}
}

/* A routing function to call the appropriate template implementation based on run-time configuration options. */
void route_gpsMaxAction(GpsChooserResponse& result, BeliefNode const *node, const GpsMaxRecommendationOptions& options) {
	if (options.searchType == GpsMaxRecommendationOptions::COMPASS) {
		route_gpsMaxAction_compass<1>(result, node, options);
	} else if (options.searchType == GpsMaxRecommendationOptions::GOLDEN) {
		route_gpsMaxAction_golden(result, node, options);
	} else {
		class UnknownSearchType: public std::exception {
			virtual const char* what() const noexcept {
				return "the GPS search type you requested is unknown. Please change your options.";
			}
		} e;
		throw e;
	}
}



} // namespace gps_detail



GpsChooserResponse gps_ucb_action(BeliefNode const* node, const Model& model, const GpsChooserOptions& options) {
	GpsChooserResponse result;
	gps_detail::route_gpsUcbAction(result, node, model, options);
	return std::move(result);
}

GpsChooserResponse gps_max_action(BeliefNode const* node, const GpsMaxRecommendationOptions& options) {
	GpsChooserResponse result;
	gps_detail::route_gpsMaxAction(result, node, options);
	return std::move(result);
}


} /* namespace choosers */
} /* namespace abt */
