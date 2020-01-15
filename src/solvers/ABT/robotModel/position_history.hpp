/** @file position_history.hpp
 *
 * Defines a class to keep track of the position of the robot in RockSample.
 *
 * This is useful, since the position is fully observable but is not included in observations.
 */
#ifndef ROBOT_POSITION_HISTORY_HPP_
#define ROBOT_POSITION_HISTORY_HPP_

#include <memory>
#include <vector>

#include "solvers/ABT/solver/abstract-problem/HistoricalData.hpp"

#include "solvers/ABT/solver/serialization/TextSerializer.hpp"

//#include "problems/shared/GridPosition.hpp"
//#include "CartesianCoordinates.hpp"
#include "RobotModel.hpp"
#include "RobotAction.hpp"
#include <iostream>

namespace robot
{
class RobotAction;
class RobotModel;

/** An implementation of the serialization methods for the PositionData class. */
class PositionDataTextSerializer : virtual public abt::TextSerializer
{
public:
    void saveHistoricalData(abt::HistoricalData const* data, std::ostream& os) override;
    std::unique_ptr<abt::HistoricalData> loadHistoricalData(std::istream& is) override;
};

/** A class to store the robot position associated with a given belief node.
 *
 * Since the robot position in RockSample is fully observable, all particles in any belief will
 * in fact have the same position, which is stored here.
 */
class HistEntry : public abt::HistoricalData
{
public:
    HistEntry();

    HistEntry(oppt::ActionSharedPtr &parentAction, 
        oppt::ObservationSharedPtr &parentObservation, 
        const unsigned int &stepNum);

    virtual ~HistEntry() = default;

    /** Creates a copy of this HistoricalData instance. */
    virtual std::unique_ptr<abt::HistoricalData> copy() const override;

    /** Generates a new child HistoricalData for a new belief node, based on the action taken
     * and observation received in going to that child node.
     */
    virtual std::unique_ptr<abt::HistoricalData> createChild(abt::Action const &action,
            abt::Observation const &observation) const override;

    oppt::ActionSharedPtr getParentAction() const;

    oppt::ObservationSharedPtr getParentObservation() const;

    unsigned int getCurrentStep() const;
private:
    oppt::ActionSharedPtr parentAction_ = nullptr;

    oppt::ObservationSharedPtr parentObservation_ = nullptr;

    unsigned int currentStep_ = 0;
};


} /* namespace manipulator */

#endif /* MANIPULATOR_POSITION_HISTORY_HPP_ */
