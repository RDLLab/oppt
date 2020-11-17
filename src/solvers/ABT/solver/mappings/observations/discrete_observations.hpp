/** @file discrete_observations.hpp
 *
 * Provides an implementation of the observation mapping interface that is designed for a space
 * of discrete observations, which could potentially be relatively large but only sparsely used.
 *
 * This is achieved by storing the mapping as a hash table (std::unordered_set), in which the
 * observations can be looked up via a hash function. Doing so allows entries to be stored only
 * for the observations that are actually encountered.
 */
#ifndef SOLVER_DISCRETE_OBSERVATIONS_HPP_
#define SOLVER_DISCRETE_OBSERVATIONS_HPP_

#include <iostream>
#include <memory>
#include <unordered_map>

#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"

#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

namespace abt {
class ActionPool;
class DiscreteObservationMapEntry;

/** An implementation of the ObservationPool interface that is based on a discrete observation
 * space.
 *
 * All of the information is stored inside the individual mapping classes, so this class serves as
 * a simple factory for instances of DiscreteObservationMap.
 */
class DiscreteObservationPool: public abt::ObservationPool {
  public:
    /** Creates a new DiscreteObservationPool. */
    DiscreteObservationPool(Solver *solver);
    virtual ~DiscreteObservationPool() = default;
    _NO_COPY_OR_MOVE(DiscreteObservationPool);

    virtual std::unique_ptr<ObservationMapping> createObservationMapping(ActionNode *owner) override;
  private:
    /** The solver. */
    Solver *solver_;
};

/** A concrete class implementing ObservationMapping for a discrete set of observations.
 *
 * The mapping entries are stored in a hash table (std::unordered_set), which maps each observation
 * to its associated entry in the mapping, or nullptr if there is no entry yet.
 */
class DiscreteObservationMap: public abt::ObservationMapping {
  public:
    friend class DiscreteObservationMapEntry;
    friend class DiscreteObservationTextSerializer;

    /** Creates a new, empty discrete observation map for the given ActionNode. */
    DiscreteObservationMap(ActionNode *owner, Solver *solver);
    virtual ~DiscreteObservationMap() = default;
    _NO_COPY_OR_MOVE(DiscreteObservationMap);

    /* -------------- Access to and management of child nodes. ---------------- */
    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;
    virtual long getNChildren() const override;

    virtual void deleteChild(ObservationMappingEntry const *entry) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual std::vector<ObservationMappingEntry const *> getChildEntries() const override;
    virtual ObservationMappingEntry *getEntry(Observation const &obs) override;
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const override;

    /* ------------- Methods for accessing visit counts. --------------- */
    virtual long getTotalVisitCount() const override;

  private:
    /** The solver. */
    Solver *solver_;

    /** A hashing operator that works on unique_ptr<Observation> by calling the virtual hash method
     * of the observation itself.
     */
    struct HashContents {
        std::size_t operator()(std::unique_ptr<Observation> const &obs) const {
            return obs->hash();
        }
    };

    /** An equality operator compares unique_ptr<Observation> by calling the virtual equals() method
     * of the observation.
     */
    struct EqualContents {
        std::size_t operator()(std::unique_ptr<Observation> const &o1,
                std::unique_ptr<Observation> const &o2) const {
            return o1->equals(*o2);
        }
    };

    /** A typedef to make the syntax for this mapping type less verbose. */
    typedef std::unordered_map<std::unique_ptr<Observation>,
            std::unique_ptr<DiscreteObservationMapEntry>,
            HashContents, EqualContents> ChildMap;

    /** The mapping of observations to their entries. */
    ChildMap childMap_;

    /** The total visit count for all of the entries. */
    long totalVisitCount_;
};


/** A concrete class implementing ObservationMappingEntry for a discrete set of observations.
 *
 * Each entry stores a pointer back to its parent map, the actual observation it is associated with,
 * and its child node and visit count.
 */
class DiscreteObservationMapEntry : public abt::ObservationMappingEntry {
    friend class DiscreteObservationMap;
    friend class DiscreteObservationTextSerializer;

public:
    virtual ObservationMapping *getMapping() const override;
    virtual std::unique_ptr<Observation> getObservation() const override;
    virtual BeliefNode *getBeliefNode() const override;
    virtual long getVisitCount() const override;

    virtual void updateVisitCount(long deltaNVisits) override;

private:
    /** The parent mapping for this entry. */
    DiscreteObservationMap *map_ = nullptr;
    /** The observation this entry is tied to. */
    std::unique_ptr<Observation> observation_ = nullptr;
    /** The child node of this entry (should always be non-null). */
    std::unique_ptr<BeliefNode> childNode_ = nullptr;
    /** The visit count for this entry. */
    long visitCount_ = 0;
};


/** A partial implementation of the Serializer interface which provides serialization methods for
 * the above discrete observation mapping classes.
 */
class DiscreteObservationTextSerializer: virtual public abt::Serializer {
  public:
    DiscreteObservationTextSerializer() = default;
    virtual ~DiscreteObservationTextSerializer() = default;
    _NO_COPY_OR_MOVE(DiscreteObservationTextSerializer);

    virtual void saveObservationPool(
            ObservationPool const &observationPool, std::ostream &os) override;
    virtual std::unique_ptr<ObservationPool> loadObservationPool(
            std::istream &is) override;
    virtual void saveObservationMapping(ObservationMapping const &map,
            std::ostream &os) override;
    virtual std::unique_ptr<ObservationMapping> loadObservationMapping(ActionNode *owner,
            std::istream &is) override;
};
} /* namespace abt */

#endif /* SOLVER_DISCRETE_OBSERVATIONS_HPP_ */
