/** @file position_history.cpp
 *
 * Contains the implementations for PositionData and PositionDataTextSerializer.
 */
#include "position_history.hpp"

#include <iostream>
#include <sstream>


#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

namespace robot
{

HistEntry::HistEntry():
  abt::HistoricalData() {

}

HistEntry::HistEntry(oppt::ActionSharedPtr &parentAction,
                     oppt::ObservationSharedPtr &parentObservation, 
                     const unsigned int &stepNum):
  abt::HistoricalData(),
  parentAction_(parentAction),
  parentObservation_(parentObservation),
  currentStep_(stepNum) {

}

oppt::ActionSharedPtr HistEntry::getParentAction() const {
  return parentAction_;
}

oppt::ObservationSharedPtr HistEntry::getParentObservation() const {
  return parentObservation_;
}

unsigned int HistEntry::getCurrentStep() const {
  return currentStep_;
}

/* ---------------------- PositionData --------------------- */
std::unique_ptr<abt::HistoricalData> HistEntry::copy() const {
  ERROR("DO COPY");
  return nullptr;
}

/** Generates a new child HistoricalData for a new belief node, based on the action taken
 * and observation received in going to that child node.
 */
std::unique_ptr<abt::HistoricalData> HistEntry::createChild(abt::Action const &action,
    abt::Observation const &observation) const {
  auto opptAction = static_cast<DiscreteRobotAction const &>(action).getOpptAction();
  auto opptObservation = static_cast<shared::RobotObservation const &>(observation).getOpptObservation();
  std::unique_ptr<abt::HistoricalData> childHistEntry(new HistEntry(opptAction, opptObservation, currentStep_ + 1));
  return std::move(childHistEntry);

}

/* --------------------- PositionDataTextSerializer -------------------- */
void PositionDataTextSerializer::saveHistoricalData(abt::HistoricalData const* data,
    std::ostream& os)
{
  /**PositionData const& position_data = static_cast<PositionData const&>(*data);
  position_data.getRobotState()->serialize(os);
  os << endl;
  os << position_data.getCurrentStep() << " END" << endl;*/
}

std::unique_ptr<abt::HistoricalData> PositionDataTextSerializer::loadHistoricalData(
  std::istream& is)
{
  /**std::string line;
  std::getline(is, line);
  RobotModel* model = static_cast<RobotModel*>(getModel());
  RobotStateSharedPtr robotState = model->getRobotEnvironment()->getRobot()->getSerializer()->loadState(line);
  std::getline(is, line);

  int currentStep;
  std::istringstream ss(line);
  ss >> currentStep;
  return std::make_unique<PositionData>(model, robotState, nullptr, currentStep, true);*/
  return nullptr;
}

} /* namespace manipulator */
