/** @file approximate_observations.hpp
 *
 * Provides an implementation of the observation mapping interface that is designed for
 * continuous observation spaces.
 *
 * WARNING: This implementation is currently quite rough; it needs better solvers and data
 * structures (e.g. some kind of algorithm for dynamic clustering of observations).
 *
 * Currently, it works by storing the observation entries in a vector, and comparing every new
 * observation to all of the previous ones - this is clearly rather inefficient.
 */
#ifndef SOLVER_APPROXIMATE_OBSERVATIONS_HPP_
#define SOLVER_APPROXIMATE_OBSERVATIONS_HPP_

#include <memory>
#include <vector>

#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"

#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

namespace abt {
class ApproximateObservationMapEntry;
class ActionPool;
class BeliefNode;

/** An implementation of the ObservationPool interface that is based on a continuous observation
 * space.
 *
 * This class has a single field, which defines the maximum distance between an observation and
 * the base observation in order for them to be grouped together. In order for this to work, the
 * observation class also needs to implement the distanceTo() method in a meaningful way.
 *
 * Note that since the maximum distance is taken between each observation and the representative
 * observation for that entry, the actual maximum distance between any two observations that are
 * grouped together is 2 * maxDistance.
 */
class ApproximateObservationPool: public abt::ObservationPool {
  public:
    /** Creates a new observation pool; the individual mappings created will group together
     * observations based on the given radius value.
     */
    ApproximateObservationPool(Solver *solver, FloatType maxDistance);
    virtual ~ApproximateObservationPool() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationPool);

    virtual std::unique_ptr<ObservationMapping> createObservationMapping(ActionNode *owner) override;

  protected:
    /** The solver. */
    Solver *solver_;
    /** The maximum radius for observations to be grouped together. */
    FloatType maxDistance_;
};

/** A concrete class implementing ObservationMapping for a continuous set of observations.
 *
 * The mapping entries are stored in a vector; entries are then looked up by iterating over the
 * vector and finding the first entry that is within the maximum distance.
 *
 * In effect, each entry is a sphere in the observation space, which is centered at the first
 * observation used to make that entry, and has a radius of the given maximum distance.
 *
 * Note that this is sensitive to the order in which the entries are stored, because there may
 * be more than one entry that meets the distance criterion for a given observation.
 */
class ApproximateObservationMap: public abt::ObservationMapping {
  public:
    friend class ApproximateObservationMapEntry;
    friend class ApproximateObservationTextSerializer;

    /** Creates a new ApproximateObservationMap which will be owned by the given ActionNode, and
     * for which the maximum distance for an entry to "match" is the given distance.
     */
    ApproximateObservationMap(ActionNode *owner, Solver *solver, FloatType maxDistance);
    virtual ~ApproximateObservationMap() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationMap);

    /* -------------- Access to and management of child nodes. ---------------- */
    virtual BeliefNode *getBelief(Observation const &obs) const override;
    virtual BeliefNode *createBelief(Observation const &obs) override;
    virtual long getNChildren() const override;

    virtual void deleteChild(ObservationMappingEntry const *entry) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual std::vector<ObservationMappingEntry const *> getChildEntries() const override;
    virtual ObservationMappingEntry *getEntry(Observation const &obs) override;
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const override;

    /* --------------- Methods for accessing visit counts. ----------------- */
    virtual long getTotalVisitCount() const override;

  protected:
    /** The solver. */
    Solver *solver_;

    /** The maximum distance for an observation to count as part of an entry. */
    FloatType maxDistance_;
    /** The vector of entries for this mapping. */
    std::vector<std::unique_ptr<ApproximateObservationMapEntry>> entries_;

    /** The total number of visits over all of the entries in this mapping. */
    long totalVisitCount_;
};

/** A concrete class implementing ObservationMappingEntry for a continuous set of observations.
 *
 * Each entry stores a pointer back to its parent map, the actual observation it is associated with,
 * and its child node and visit count.
 */
class ApproximateObservationMapEntry : public abt::ObservationMappingEntry {
    friend class ApproximateObservationMap;
    friend class ApproximateObservationTextSerializer;
public:
    virtual ~ApproximateObservationMapEntry() {}
    virtual ObservationMapping *getMapping() const override;
    virtual std::unique_ptr<Observation> getObservation() const override;
    virtual BeliefNode *getBeliefNode() const override;
    virtual long getVisitCount() const override;

    virtual void updateVisitCount(long deltaNVisits) override;

protected:
    /** The parent mapping of this entry. */
    ApproximateObservationMap *map_ = nullptr;
    /** The observation associated with this entry => the center point of this n-sphere.  */
    std::unique_ptr<Observation> observation_ = nullptr;
    /** The child action node for this entry. */
    std::unique_ptr<BeliefNode> childNode_ = nullptr;
    /** The number of visits for this entry. */
    long visitCount_ = 0;
};

/** A partial implementation of the Serializer interface which provides serialization methods for
 * the above approximate observation mapping classes.
 */
class ApproximateObservationTextSerializer: virtual public abt::Serializer {
  public:
    ApproximateObservationTextSerializer() = default;
    virtual ~ApproximateObservationTextSerializer() = default;
    _NO_COPY_OR_MOVE(ApproximateObservationTextSerializer);

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

#endif /* SOLVER_APPROXIMATE_OBSERVATIONS_HPP_ */
