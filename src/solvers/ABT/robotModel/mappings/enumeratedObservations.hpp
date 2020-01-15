#ifndef _OPPT_ABT_ENUMERATED_OBSERVATIONS_HPP_
#define _OPPT_ABT_ENUMERATED_OBSERVATIONS_HPP_
#include "oppt/opptCore/core.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/serialization/Serializer.hpp"

#include "solvers/ABT/solver/mappings/observations/ObservationPool.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationMapping.hpp"
#include "solvers/ABT/solver/abstract-problem/DiscretizedPoint.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationMappingEntry.hpp"
#include "solvers/ABT/solver/mappings/observations/enumerated_observations.hpp"

namespace oppt
{

class EnumeratedABTObservationMapEntry;

class EnumeratedABTObservationPool: public abt::ObservationPool
{
public:
    /** Creates a new EnumeratedObservationPool with the given set of observations, in the given
     * order.
     */
    EnumeratedABTObservationPool(abt::Solver* solver,
                                   std::vector<std::unique_ptr<abt::DiscretizedPoint>> observations);
    virtual ~EnumeratedABTObservationPool() = default;
    _NO_COPY_OR_MOVE(EnumeratedABTObservationPool);

    /** Creates a new observation mapping for the given ActionNode. */
    virtual std::unique_ptr<abt::ObservationMapping> createObservationMapping(abt::ActionNode* owner) override;

private:
    /** The solver. */
    abt::Solver* solver_;
    /** All of the observations in their enumerated order. */
    std::vector<std::unique_ptr<abt::DiscretizedPoint>> observations_;
};

class EnumeratedABTObservationMap: public abt::ObservationMapping
{
public:
    friend class EnumeratedABTObservationMapEntry;
    EnumeratedABTObservationMap(abt::ActionNode* owner, abt::Solver* solver,
                                  std::vector<std::unique_ptr<abt::DiscretizedPoint>> const& allObservations);

    // Default destructor; copying and moving disallowed!
    virtual ~EnumeratedABTObservationMap() = default;
    _NO_COPY_OR_MOVE(EnumeratedABTObservationMap);

    virtual abt::BeliefNode* getBelief(abt::Observation const& obs) const override;
    virtual abt::BeliefNode* createBelief(abt::Observation const& obs) override;

    virtual long getNChildren() const override;

    virtual void deleteChild(abt::ObservationMappingEntry const* entry) override;

    /* -------------- Retrieval of mapping entries. ---------------- */
    virtual std::vector<abt::ObservationMappingEntry const*> getChildEntries() const override;
    virtual abt::ObservationMappingEntry* getEntry(abt::Observation const& obs) override;
    virtual abt::ObservationMappingEntry const* getEntry(abt::Observation const& obs) const override;


    /* ------------- Methods for accessing visit counts. --------------- */
    virtual long getTotalVisitCount() const override;

protected:
    /** The solver. */
    abt::Solver* solver_;

    /** A reference back to the vector of observations in the pool. */
    std::vector<std::unique_ptr<abt::DiscretizedPoint>> const& allObservations_;

    /** The number of observations in this mapping. */
    long nObservations_;
    /** An array of all of the child entries in this mapping. */
    std::unique_ptr<EnumeratedABTObservationMapEntry[]> entries_;

    /** The number of child nodes for this mapping - not all entries will have a child node! */
    long nChildren_;
    /** The total visit count over all observations in this mapping. */
    long totalVisitCount_;

};

class EnumeratedABTObservationMapEntry: public abt::ObservationMappingEntry
{
    friend class EnumeratedABTObservationMap;
public:
    virtual ~EnumeratedABTObservationMapEntry() {}
    virtual abt::ObservationMapping *getMapping() const override;
    virtual std::unique_ptr<abt::Observation> getObservation() const override;
    virtual abt::BeliefNode *getBeliefNode() const override;
    virtual long getVisitCount() const override;

    virtual void updateVisitCount(long deltaNVisits) override;

protected:
    /** The parent mapping for this entry. */
    EnumeratedABTObservationMap *map_ = nullptr;
    /** The enumeration number for this entry - this should be its index into the vector of
     * observations and also its index in the array of entries in the parent mapping.
     */
    long index_ = 0;
    /** The child node for this entry, if it has one. */
    std::unique_ptr<abt::BeliefNode> childNode_ = nullptr;
    /** The visit count for this entry. */
    long visitCount_ = 0;

};

}

#endif
