/** @file continuous_actions.hpp
 *
 * Provides a default implementation for an action mapping that uses a set of continuous or hybrid actions,
 * i.e. there is set of action categories, and the actions in each of these
 * categories will map to the same child nodes in the belief tree.
 *
 * Continuous actions need to be constructed using action construction data which is a vector representation
 * that is interpreted by the chooser.
 *
 * There is also support for additional discrete actions to be added to the set of actions.
 *
 * The mapping class stores the entries in an unordered_map indexed by the construction data for fast retrieval
 *
 *
 * This involves subclasses of the following abstract classes:
 * -ActionPool
 * -ActionMapping
 * -ActionMappingEntry
 *
 * as well as a serialization class providing methods to serialize this particular kind of
 * action pool and action mapping.
 */
#ifndef SOLVER_CONTINUOUS_ACTIONS_HPP_
#define SOLVER_CONTINUOUS_ACTIONS_HPP_

#include <memory>
#include <vector>

#include "oppt/global.hpp"
#include "solvers/ABT/solver/LinkedHashSet.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"
#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/ActionNode.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"

namespace abt {
class ContinuousActionMap;
class ContinuousActionMapEntry;

/** An abstract class that contains the data to construct continuous actions.
 *
 * The data has to be in vector form. The interface is storage and size agnostic.
 * As such the constructiondata is only accessed though the data() function which
 * returns a pointer to the array data of an std::array or std::vector or similar.
 *
 * The data() pointer only needs to point to the part relevant for the continuous
 * action space. Additional discrete actions may be handled otherwise.
 *
 * Note, that a chooser for continuous actions is likely to make assumptions about
 * the size of data(). It is expected that there is one value for each dimension.
 *
 */
class ContinuousActionConstructionDataBase {
public:
	virtual ~ContinuousActionConstructionDataBase() = default;

	/** Returns a pointer to the data array of the underlying vector. If implemented using an std::array, just return std::array::data() */
	virtual const FloatType* data() const = 0;
};



/** An abstract class for continuous actions.
 *
 * An implementation should keep a copy of the construction data so a reference to it can
 * be provided when needed.
 */
class ContinuousAction: public abt::Action {
public:
    virtual ~ContinuousAction() {}
	virtual const ContinuousActionConstructionDataBase& getConstructionData() const = 0;
};


/** An abstract service class for ContinuousActionMapp to store actions.
 *
 * Actions are stored in this container and indexed by the construction data.
 *
 * This is meant to be implemented as unodered_map.
 *
 * Implementations can tweak the hashing and equality functions used to create an
 * equivalence relation for very similar actions.
 *
 */
class ContinuousActionContainerBase {
public:
	ContinuousActionContainerBase() = default;
	virtual ~ContinuousActionContainerBase() = default;
	_NO_COPY_OR_MOVE(ContinuousActionContainerBase);

	/** like std::unordered_map::size() */
	virtual size_t size() const = 0;

	/** like std::unordered_map::at() */
	virtual std::unique_ptr<ContinuousActionMapEntry>& at(const ContinuousActionConstructionDataBase& key) = 0;

	/** like std::unordered_map::at() */
	virtual const std::unique_ptr<ContinuousActionMapEntry>& at(const ContinuousActionConstructionDataBase& key) const = 0;

	/** like std::unordered_map::operator[] */
	virtual std::unique_ptr<ContinuousActionMapEntry>& operator[](const ContinuousActionConstructionDataBase& key) = 0;

	/** like std::unordered_map::at() but only providing the data pointer for the construction data */
	virtual std::unique_ptr<ContinuousActionMapEntry>& at(const FloatType* constructionDataVector) = 0;

	/** like std::unordered_map::at() but only providing the data pointer for the construction data  */
	virtual const std::unique_ptr<ContinuousActionMapEntry>& at(const FloatType* constructionDataVector) const = 0;

	/** like std::unordered_map::operator[] but only providing the data pointer for the construction data  */
	virtual std::unique_ptr<ContinuousActionMapEntry>& operator[](const FloatType* constructionDataVector) = 0;

	/** like std::unordered_map::clear() */
	virtual void clear() = 0;

	/** get all entries of the container */
	virtual std::vector<ActionMappingEntry const*> getEntries() const = 0;

	/** get all entries of the container with children.
	 *
	 * The default implementation uses getEntries() and then selects those with children.
	 * This works, but an implementation may choose to optimise the performance.
	 */
	virtual std::vector<ActionMappingEntry const*> getEntriesWithChildren() const;

	/** get all entries of the container that are visited.
	 *
	 * The default implementation uses getEntries() and then selects those that are visited.
	 * This works, but an implementation may choose to optimise the performance.
	 */
	virtual std::vector<ActionMappingEntry const*> getEntriesWithNonzeroVisitCount() const;


};


/** An implementation of ContinuousActionContainerBase as template.
 *
 * It uses CONSTRUCTION_DATA::hash() and CONSTRUCTION_DATA::equal() to compare the keys.
 */
template<class CONSTRUCTION_DATA>
class ContinuousActionContainer: public ContinuousActionContainerBase {
public:
	typedef CONSTRUCTION_DATA KeyType;
	typedef typename KeyType::HashEqualOptions HashEqualOptions;
private:
	/** Service class so the unordered_map can access hash() and equal(). */
	class Comparator {
		HashEqualOptions options;
	public:
		Comparator(const HashEqualOptions& theOptions): options(theOptions) {};

		size_t operator()(const KeyType& key) const { return key.hash(options); }
		size_t operator()(const KeyType& first, const KeyType& second) const { return first.equal(second, options); }
	};

public:
	ContinuousActionContainer(const HashEqualOptions& options): container(0, Comparator(options), Comparator(options)) {};
	virtual ~ContinuousActionContainer(){}
	_NO_COPY_OR_MOVE(ContinuousActionContainer);

	virtual size_t size() const override;
	virtual std::unique_ptr<ContinuousActionMapEntry>& at(const ContinuousActionConstructionDataBase& key) override;
	virtual const std::unique_ptr<ContinuousActionMapEntry>& at(const ContinuousActionConstructionDataBase& key) const override;
	virtual std::unique_ptr<ContinuousActionMapEntry>& operator[](const ContinuousActionConstructionDataBase& key) override;
	virtual std::unique_ptr<ContinuousActionMapEntry>& at(const FloatType* constructionDataVector) override;
	virtual const std::unique_ptr<ContinuousActionMapEntry>& at(const FloatType* constructionDataVector) const override;
	virtual std::unique_ptr<ContinuousActionMapEntry>& operator[](const FloatType* constructionDataVector) override;
	virtual void clear() override;
	virtual std::vector<ActionMappingEntry const*> getEntries() const override;
	virtual std::vector<ActionMappingEntry const*> getEntriesWithChildren() const override;
	virtual std::vector<ActionMappingEntry const*> getEntriesWithNonzeroVisitCount() const override;
private:
	std::unordered_map<CONSTRUCTION_DATA, std::unique_ptr<ContinuousActionMapEntry>, Comparator, Comparator> container = {};
};





/** An abstract implementation of the ActionPool interface that considers continuous actions
 *
 * A concrete implementation of this abstract class requires implementations for ...
 */
class ContinuousActionPool: public abt::ActionPool {
	friend class ContinuousActionMap;
public:
	typedef abt::Action Action;
	typedef abt::BeliefNode BeliefNode;
	typedef abt::ContinuousActionContainerBase ContinuousActionContainerBase;
	typedef abt::ContinuousActionConstructionDataBase ContinuousActionConstructionDataBase;
public:
	ContinuousActionPool() = default;
	virtual ~ContinuousActionPool() = default;
	_NO_COPY_OR_MOVE(ContinuousActionPool);


	/** Returns a ContinuousActionMap for the given belief node. */
	virtual std::unique_ptr<ActionMapping> createActionMapping(BeliefNode *node) override;

	/** Returns a container to store actions within a ContinuousActionMap */
	virtual std::unique_ptr<ContinuousActionContainerBase> createActionContainer(BeliefNode *node) const = 0;

	/** Returns an action construction data object based on a vector of numbers that was provided.
	 *
	 * Here, constructionData is a pointer to a data array as it is returned by
	 * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
	 * create new actions based on values it seems fit.
	 */
	virtual std::unique_ptr<ContinuousActionConstructionDataBase> createActionConstructionData(const FloatType* constructionDataVector, const BeliefNode* belief) const = 0;

	/** Returns an action based on the Construction Data that was provided.
	 *
	 * In this version, constructionData is a pointer to a data array as it is returned by
	 * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
	 * create new actions based on values it seems fit.
	 *
	 * The default version uses createActionConstructionData first and then creates an action based
	 * on the full construction data. This might be inefficient and an implementation can override
	 * this function for a more direct approach.
	 *
	 * TODO: Check whether this function is actually used or can be removed.
	 */
	virtual std::unique_ptr<Action> createAction(const FloatType* constructionDataVector, const BeliefNode* belief) const;


	/** Returns an action based on the Construction Data that was provided.
	 *
	 * The default version calls createAction(constructionData.data()) which is probably fine
	 * in a purely continuous case, but probably not in a hybrid case.
	 */
	virtual std::unique_ptr<Action> createAction(const ContinuousActionConstructionDataBase& constructionData) const = 0;


	/** Returns the initial bounding box for the continuous search.
	 *
	 * For each dimension, the first entry of the pair is the lower bound, the second entry is the upper bound.
	 */
	virtual std::vector<std::pair<FloatType, FloatType>> getInitialBoundingBox(BeliefNode* belief) const = 0;

	/** Returns a shared pointer to a container containing the construction data for the additional fixed actions in a hybrid action space.
	 *
	 * The result is a shared pointer. Thus, the implementation can decide whether it wants to create the container and pass on ownership or it
	 * can return a reference to an internal vector without having to re-create it every time.
	 *
	 * The default version returns null to indicate there are no fixed actions.
	 */
	virtual std::vector<std::unique_ptr<ContinuousActionConstructionDataBase>> createFixedActions(const BeliefNode* belief) const;


	/** This acts as a hint whether the chooser should try the fixed actions in the sequence they are given
	 * or randomise their order.
	 *
	 * It acts as a hint only and it depends on the chooser whether this option has any effect.
	 *
	 * The default version always returns true. (randomise actions)
	 */
	virtual bool randomiseFixedActions(const BeliefNode* belief) const;

};



/** The real base class for ChooserDataBase.
 *
 * Do not implement this, but ChooserDataBase instead so serialisation works.
 */
class ChooserDataBaseBase {
	typedef ChooserDataBaseBase This;
	typedef ContinuousActionMap ThisActionMap;
	typedef ContinuousActionMapEntry ThisActionMapEntry;
public:
	ChooserDataBaseBase() = default;
	virtual ~ChooserDataBaseBase() = default;
	_NO_COPY_OR_MOVE(ChooserDataBaseBase);

	/** This function can return a marker attached with an action.
	 *
	 * Implementing something useful is optional (the default returns an empty string.
	 * This can be used for debugging and/or profiling.
	 */
	virtual std::string getActionMarker(const ThisActionMapEntry* entry) const;

	void saveToStream(std::ostream& os, const ThisActionMap& map) const;
	static std::unique_ptr<This> loadFromStream(std::istream& is, ThisActionMap& map);
protected:

	typedef std::function<std::unique_ptr<ChooserDataBaseBase>(std::istream&, ThisActionMap& map)> LoadFromStreamFunction;
	static void registerDerivedType(const std::string& name, const LoadFromStreamFunction& loader);

	virtual void saveToStream_real(std::ostream& os, const ThisActionMap& map) const = 0;

private:
	static std::unordered_map<std::string, LoadFromStreamFunction>& getDerivedLoadersSingleton();
};



/** A base class to hold data for the chooser.
 *
 * An implementation of this data structure can be stored in a continuous action map.
 * Its use it at the chooser's discresion.
 *
 * The action map will take care of serialisation and destruction.
 *
 * implementation are expected to be constructible from std::istream for de-serialisation.
 *
 */
template<class Derived>
class ChooserDataBase: public ChooserDataBaseBase {
	typedef ChooserDataBase This;
	typedef ChooserDataBaseBase Base;
public:
	ChooserDataBase() = default;
	virtual ~ChooserDataBase() = default;
	_NO_COPY_OR_MOVE(ChooserDataBase);

private:
	/** This function registers "Derived" with the base class
	 *
	 * This is needed for de-serialisation. It is then possible to load chooser data from file
	 * regardless of its type by having the type name written to the file.
	 */
	static void registerType();

	/** Only a dummy variable. It is used to run registerType() at program start. */
	static bool initialisationDummy;

	/** A useless function that only accesses initialisationDummy.
	 *
	 * If initialisationDummy is used nowhere, it isn't instantiated and registerType() doesn't run.
	 */
	virtual void accessInitialisationDummy();
};



/** A concrete class implementing ActionMapping for a continuous or hybrid action space.
 *
 * This class stores its mapping entries in an unordered_map for easy access. In addition
 * it allows the chooser to store and access additional data.
 */
class ContinuousActionMap: public abt::ActionMapping {
private:
	typedef ContinuousActionMap This;
	typedef ContinuousActionMapEntry ThisActionMapEntry;
	typedef ContinuousAction ThisAction;
	typedef ContinuousActionConstructionDataBase ThisActionConstructionData;
public:
	friend class ContinuousActionMapEntry;
	friend class ContinuousActionTextSerializer;

	/** Constructs a new DiscretizedActionMap, which will be owned by the given belief node,
	 * and be associated with the given DiscretizedActionpool.
	 *
	 */
	ContinuousActionMap(BeliefNode *owner, ContinuousActionPool *pool);

	// Default destructor; copying and moving disallowed!
	virtual ~ContinuousActionMap() = default;
	_NO_COPY_OR_MOVE(ContinuousActionMap);

	/* -------------- Retrieval internal infrastructure members ---------------- */
	const ContinuousActionPool* getActionPool() const;

	/* -------------- Creation and retrieval of nodes. ---------------- */
	virtual ActionNode *getActionNode(Action const &action) const override;
	virtual ActionNode *createActionNode(Action const &action) override;
	virtual long getNChildren() const override;

	// TODO: This is not const. FloatType check the interface. I think this should be changed everywhere in oppt...
	virtual void deleteChild(ActionMappingEntry const *entry) override;

	/* -------------- Retrieval of mapping entries. ---------------- */
	virtual std::vector<ActionMappingEntry const *> getChildEntries() const override;

	virtual long getNumberOfVisitedEntries() const override;
	virtual std::vector<ActionMappingEntry const *> getVisitedEntries() const override;
	virtual ActionMappingEntry *getEntry(Action const &action) override;
	virtual ActionMappingEntry const *getEntry(Action const &action) const override;

	ThisActionMapEntry* getActionMapEntry(const FloatType* constructionDataVector);
	ThisActionMapEntry* createOrGetActionMapEntry(const FloatType* constructionDataVector);

	const std::vector<ThisActionMapEntry*>& getFixedEntries() const;

	/* ----------------- Methods for unvisited actions ------------------- */
	/** Returns the next action to be tried for this node, or nullptr if there are no more. */
	virtual std::unique_ptr<Action> getNextActionToTry() override;

	/* -------------- Retrieval of general statistics. ---------------- */
	virtual long getTotalVisitCount() const override;


	/* The chooserData
	 *
	 * This class does not do much with it. It is just there as a place for the chooser
	 * to store data. Thus it is simply a public member.
	 *
	 * This class only takes care of desctruction, serialisation and de-serialisation
	 * of the data stored here. Otherwise the chooserData is left alone.
	 */
	std::unique_ptr<ChooserDataBaseBase> chooserData = nullptr;

protected:
	/** The pool associated with this mapping. */
	ContinuousActionPool *pool;

	/** The container to store the action map entries. */
	std::unique_ptr<ContinuousActionContainerBase> entries;

	/** The number of action node children that have been created. */
	long nChildren = 0;

	/** The number of entries with nonzero visit counts. */
	long numberOfVisitedEntries = 0;

	/** The total of the visit counts of all of the individual entries. */
	long totalVisitCount = 0;

	/** Stores references to the entries that are considered fixed */
	std::vector<ThisActionMapEntry*> fixedEntries;

};


/** A concrete class implementing ActionMappingEntry for a discretized action space.
 *
 * Each entry stores its bin number and a reference back to its parent map, as well as a child node,
 * visit count, total and mean Q-values, and a flag for whether or not the action is legal.
 */
class ContinuousActionMapEntry : public abt::ActionMappingEntry {

	typedef ContinuousActionMapEntry This;
	typedef ContinuousActionMap ThisActionMap;
	typedef ContinuousActionConstructionDataBase ThisActionConstructionData;


	friend class ContinuousActionTextSerializer;


public:
	ContinuousActionMapEntry(ThisActionMap* map, std::unique_ptr<ThisActionConstructionData>&& constructionData, bool isLegal = false);
	virtual ~ContinuousActionMapEntry(){}
	_NO_COPY_OR_MOVE(ContinuousActionMapEntry);

	virtual ActionMapping *getMapping() const override;
	virtual std::unique_ptr<Action> getAction() const override;
	virtual ActionNode *getActionNode() const override;
	virtual long getVisitCount() const override;
	virtual FloatType getTotalQValue() const override;
	virtual FloatType getMeanQValue() const override;
	virtual bool isLegal() const override;

	/** Returns the bin number associated with this entry. */
	long getBinNumber() const;

	virtual bool update(long deltaNVisits, FloatType deltaTotalQ) override;
	virtual void setLegal(bool legal) override;

	void setChild(std::unique_ptr<ActionNode>&& child);
	void deleteChild();
	const ActionNode* getChild() const;

	const ThisActionConstructionData& getConstructionData() const;

	virtual std::string getMarker() const override;

protected:
	/** The parent action mapping. */
	ContinuousActionMap* const map = nullptr;

	/** The construction data represented by this entry */
	std::unique_ptr<ThisActionConstructionData> constructionData;

	/** The child action node, if one exists. */
	std::unique_ptr<ActionNode> childNode = nullptr;
	/** The visit count for this edge. */
	long visitCount_ = 0;
	/** The total Q-value for this edge. */
	FloatType totalQValue_ = 0;
	/** The mean Q-value for this edge => should be equal to totalQValue_ / visitCount_ */
	FloatType meanQValue_ = 0;
	/** True iff this edge is legal. */
	bool isLegal_ = false; // Entries are illegal by default.
};

/** A partial implementation of the Serializer interface which provides serialization methods for
 * the above continuous action mapping classes.
 */
class ContinuousActionTextSerializer: virtual public abt::Serializer {
	typedef ContinuousActionTextSerializer This;
protected:
	typedef ContinuousActionMap ThisActionMap;
	typedef ContinuousActionMapEntry ThisActionMapEntry;
	typedef ContinuousActionConstructionDataBase ThisActionConstructionDataBase;
public:
	ContinuousActionTextSerializer() = default;
	virtual ~ContinuousActionTextSerializer() = default;
	_NO_COPY_OR_MOVE(ContinuousActionTextSerializer);

	virtual void saveActionPool(ActionPool const &actionPool, std::ostream &os) override;
	virtual std::unique_ptr<ActionPool> loadActionPool(std::istream &is) override;
	virtual void saveActionMapping(ActionMapping const &map, std::ostream &os) override;
	virtual std::unique_ptr<ActionMapping> loadActionMapping(BeliefNode *node, std::istream &is) override;

	/** Loads the data from the input stream into the given ThisActionMap. */
	virtual void loadActionMapping(ThisActionMap &map, std::istream &is);
protected:
	virtual void saveActionMapEntry(const ThisActionMapEntry& entry, std::ostream& os);
	virtual std::unique_ptr<ThisActionMapEntry> loadActionMapEntry(ThisActionMap& map, std::istream& is);

	/** Saves the construction data to the output stream */
	virtual void saveConstructionData(const ThisActionConstructionDataBase*, std::ostream& os) = 0;
	/** Loads the construction data from the input stream */
	virtual std::unique_ptr<ThisActionConstructionDataBase> loadConstructionData(std::istream& is) = 0;
};





/* ------------------- Template Implementations ------------------- */

/* ------------------- ContinuousActionContainer ------------------- */
template<class CONSTRUCTION_DATA>
inline size_t ContinuousActionContainer<CONSTRUCTION_DATA>::size() const {
	return container.size();
}

template<class CONSTRUCTION_DATA>
inline std::unique_ptr<ContinuousActionMapEntry>& ContinuousActionContainer<CONSTRUCTION_DATA>::at(const ContinuousActionConstructionDataBase& key) {
	return container.at(static_cast<const KeyType&>(key));
}

template<class CONSTRUCTION_DATA>
inline const std::unique_ptr<ContinuousActionMapEntry>& ContinuousActionContainer<CONSTRUCTION_DATA>::at(const ContinuousActionConstructionDataBase& key) const {
	return container.at(static_cast<const KeyType&>(key));
}

template<class CONSTRUCTION_DATA>
inline std::unique_ptr<ContinuousActionMapEntry>& ContinuousActionContainer<CONSTRUCTION_DATA>::operator[](const ContinuousActionConstructionDataBase& key) {
	return container[static_cast<const KeyType&>(key)];
}

template<class CONSTRUCTION_DATA>
inline std::unique_ptr<ContinuousActionMapEntry>& ContinuousActionContainer<CONSTRUCTION_DATA>::at(const FloatType* constructionDataVector) {
	return container.at(static_cast<const KeyType&>(KeyType(constructionDataVector)));
}

template<class CONSTRUCTION_DATA>
inline const std::unique_ptr<ContinuousActionMapEntry>& ContinuousActionContainer<CONSTRUCTION_DATA>::at(const FloatType* constructionDataVector) const {
	return container.at(static_cast<const KeyType&>(KeyType(constructionDataVector)));
}

template<class CONSTRUCTION_DATA>
inline std::unique_ptr<ContinuousActionMapEntry>& ContinuousActionContainer<CONSTRUCTION_DATA>::operator[](const FloatType* constructionDataVector) {
	return container[static_cast<const KeyType&>(KeyType(constructionDataVector))];
}

template<class CONSTRUCTION_DATA>
inline void ContinuousActionContainer<CONSTRUCTION_DATA>::clear() {
	container.clear();
}


template<class CONSTRUCTION_DATA>
inline std::vector<ActionMappingEntry const*> ContinuousActionContainer<CONSTRUCTION_DATA>::getEntries() const {
	std::vector<ActionMappingEntry const *> result;
	result.reserve(container.size());
	for(auto& i : container) {
		result.push_back(i.second.get());
	}
	return std::move(result);
}

template<class CONSTRUCTION_DATA>
inline std::vector<ActionMappingEntry const*> ContinuousActionContainer<CONSTRUCTION_DATA>::getEntriesWithChildren() const {
	std::vector<ActionMappingEntry const *> result;
	// let's assume most of them have children.
	result.reserve(container.size());
	for(auto& i : container) {
		ContinuousActionMapEntry const &entry = *(i.second);
		if (entry.getChild() != nullptr) {
			result.push_back(&entry);
		}
	}
	return std::move(result);
}

template<class CONSTRUCTION_DATA>
inline std::vector<ActionMappingEntry const*> ContinuousActionContainer<CONSTRUCTION_DATA>::getEntriesWithNonzeroVisitCount() const {
	std::vector<ActionMappingEntry const *> result;
	// let's assume most of them have been visited.
	result.reserve(container.size());
	for(auto& i : container) {
		ContinuousActionMapEntry const &entry = *(i.second);
		if (entry.getVisitCount() > 0) {
			if (!entry.isLegal()) {
				debug::show_message("WARNING: Illegal entry with nonzero visit count!");
			}
			result.push_back(&entry);
		}
	}
	return std::move(result);
}


/* ------------------- ChooserDataBase ------------------- */
template<class Derived>
bool ChooserDataBase<Derived>::initialisationDummy = (ChooserDataBase<Derived>::registerType(), true);

template<class Derived>
inline void ChooserDataBase<Derived>::registerType() {

	LoadFromStreamFunction loaderFunction = [](std::istream& is, ContinuousActionMap& map) {
		return Derived::loadFromStream(is, map);
	};

	registerDerivedType(typeid(Derived).name(), loaderFunction);

}

template<class Derived>
void ChooserDataBase<Derived>::accessInitialisationDummy() {
	// make sure initialisationDummy is accessed.
	if (initialisationDummy) {}
}

} /* namespace abt */

#endif /* SOLVER_CONTINUOUS_ACTIONS_HPP_ */
