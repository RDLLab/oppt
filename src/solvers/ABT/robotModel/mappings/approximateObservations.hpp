#ifndef __APPROXIMATE_OBSERVATIONS_ABT_HPP__
#define __APPROXIMATE_OBSERVATIONS_ABT_HPP__
#include "oppt/opptCore/core.hpp"
#include "solvers/ABT/solver/mappings/observations/approximate_observations.hpp"

namespace shared
{
class RobotSerializer;
}

namespace oppt
{
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
class ApproximateABTObservationPool: public abt::ApproximateObservationPool
{
public:
    /** Creates a new observation pool; the individual mappings created will group together
     * observations based on the given radius value.
     */
    ApproximateABTObservationPool(abt::Solver* solver, FloatType maxDistance);
    virtual ~ApproximateABTObservationPool() = default;
    _NO_COPY_OR_MOVE(ApproximateABTObservationPool);

    virtual std::unique_ptr<abt::ObservationMapping> createObservationMapping(abt::ActionNode* owner) override;
};

class ApproximateABTObservationMap: public abt::ApproximateObservationMap
{
public:
    friend class ApproximateObservationMapEntry;
    friend class ApproximateObservationTextSerializer;
    friend class shared::RobotSerializer;

    /** Creates a new ApproximateObservationMap which will be owned by the given ActionNode, and
     * for which the maximum distance for an entry to "match" is the given distance.
     */
    ApproximateABTObservationMap(abt::ActionNode* owner, abt::Solver* solver, FloatType maxDistance);
    virtual ~ApproximateABTObservationMap() = default;
    _NO_COPY_OR_MOVE(ApproximateABTObservationMap);

    virtual abt::BeliefNode* createBelief(abt::Observation const& obs) override;

private:
    void setTotalVisitCount(long totalVisitCount);
    
    void addToEntries(std::unique_ptr<abt::ApproximateObservationMapEntry> entry);

};

class ApproximateABTObservationMapEntry: public abt::ApproximateObservationMapEntry
{
public:
    friend class ApproximateABTObservationMap;
    friend class shared::RobotSerializer;
    virtual ~ApproximateABTObservationMapEntry();
    
private:
    void setMapping(ApproximateABTObservationMap *mapping);
    
    void setObservation(std::unique_ptr<abt::Observation> observation);
    
    void setVisitCount(long &visitCount);        
        
    void setChildNode(std::unique_ptr<abt::BeliefNode> child);
    
};

}

#endif
