#ifndef ROBOT_SERIALIZER_HPP_
#define ROBOT_SERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr
#include <fstream>

#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/State.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"

#include "solvers/ABT/solver/serialization/TextSerializer.hpp"    // for TextSerializer
#include "solvers/ABT/solver/mappings/observations/enumerated_observations.hpp"
#include "mappings/approximateObservations.hpp"
#include "RobotModel.hpp"


namespace abt
{
class Solver;
} /* namespace abt */

namespace shared
{

/** A simple method to serialize a vector of longs to an output stream. */
void saveVector(std::vector<long> values, std::ostream& os);
/** A simple method to de-serialize a vector of longs from an input stream. */
std::vector<long> loadVector(std::istream& is);

class RobotSerializer: virtual public abt::TextSerializer,
    virtual public abt::ApproximateObservationTextSerializer
{

public:
    RobotSerializer() = default;
    RobotSerializer(abt::Solver* solver);
    virtual ~RobotSerializer() = default;
    _NO_COPY_OR_MOVE(RobotSerializer);

    void saveState(abt::State const* state, std::ostream& os) override;
    std::unique_ptr<abt::State> loadState(std::istream& is) override;

    void saveObservation(abt::Observation const* obs, std::ostream& os) override;
    std::unique_ptr<abt::Observation> loadObservation(std::istream& is) override;

    /* ------------------ Saving change sequences -------------------- */
    virtual void saveModelChange(abt::ModelChange const& change, std::ostream& os) override;
    virtual std::unique_ptr<abt::ModelChange> loadModelChange(std::istream& is) override;
    
    virtual std::unique_ptr<abt::ObservationMapping> loadObservationMapping(abt::ActionNode* owner,
            std::istream& is) override;    

};

}

#endif
