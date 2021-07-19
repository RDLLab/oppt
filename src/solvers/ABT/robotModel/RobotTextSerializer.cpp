/** @file RockSampleTextSerializer.cpp
 *
 * Contains the implementations of the serialization methods for RockSample.
 */
#include "RobotTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include "oppt/global.hpp"                     // for make_unique

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

#include "solvers/ABT/solver/mappings/actions/enumerated_actions.hpp"
#include "RobotAction.hpp"
#include "position_history.hpp"

namespace abt
{
class Solver;
} /* namespace abt */

namespace robot
{


RobotDiscreteActionTextSerializer::RobotDiscreteActionTextSerializer(abt::Solver* solver) :
    RobotSerializer(solver)
{

}

void RobotDiscreteActionTextSerializer::saveAction(abt::Action const* action, std::ostream& os)
{
    if (!action) {
        os << "NULL";
        return;
    }
    static_cast<const robot::DiscreteRobotAction*>(action)->serialize(os);
}

std::unique_ptr<abt::Action> RobotDiscreteActionTextSerializer::loadAction(std::istream& is)
{
    auto serializer =
        static_cast<robot::RobotModel*>(getSolver()->getModel())->getRobotEnvironment()->getRobot()->getSerializer()->as<oppt::RobotObjectSerializer>();
    oppt::ActionSharedPtr robotAction = serializer->loadAction(is);
    if (!robotAction)
	return nullptr;
    return std::make_unique<robot::DiscreteRobotAction>(robotAction);
}

void RobotDiscreteActionTextSerializer::saveActionMapping(abt::ActionMapping const& map,
        std::ostream& os)
{
    oppt::RobotDiscreteActionMapping const& discMap = static_cast<oppt::RobotDiscreteActionMapping const&>(map);
    os << discMap.getNumberOfVisitedEntries() << " visited actions with ";
    os << discMap.getNChildren() << " children; ";
    os << discMap.getTotalVisitCount() << " visits" << std::endl;

    os << "Untried (";
    for (auto it = discMap.getBinSequence()->begin(); it != discMap.getBinSequence()->end(); it++) {
        os << *it;
        if (std::next(it) != discMap.getBinSequence()->end()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    std::multimap<std::pair<FloatType, FloatType>, oppt::RobotDiscretizedActionMapEntry const*> entriesByValue;
    long visitedCount = 0;
    for (int i = 0; i < discMap.getNumBins(); i++) {
        oppt::RobotDiscretizedActionMapEntry const& entry = discMap.getEntries()[i];
        if (entry.getVisitCount() > 0) {
            visitedCount++;
        }
        // An entry is saved if it has a node, or a nonzero visit count.
        if (entry.getVisitCount() > 0 || entry.getActionNode() != nullptr) {
            entriesByValue.emplace(
                std::make_pair(entry.getMeanQValue(), entry.getBinNumber()), &entry);
        }
    }
    if (visitedCount != discMap.getNumberOfVisitedEntries()) {
        debug::show_message("ERROR: incorrect number of visited entries!");
        ERROR("WHAAAAAAAAAAAAAAAAT");
    }

    for (auto it = entriesByValue.rbegin(); it != entriesByValue.rend(); it++) {
        oppt::RobotDiscretizedActionMapEntry const& entry = *it->second;
        os << "Action " << entry.getBinNumber() << " (";
        saveAction(entry.getAction().get(), os);
        os << "): " << entry.getMeanQValue() << " from ";
        os << entry.getVisitCount() << " visits; total: ";
        os << entry.getTotalQValue();
        if (!entry.isLegal()) {
            os << " ILLEGAL";
        }
        abt::ActionNode* node = entry.getActionNode();
        if (node == nullptr) {
            os << " NO CHILD" << std::endl;
        } else {
            os << std::endl;
            save(*entry.getActionNode(), os);
        }
    }
}

std::unique_ptr<abt::ActionMapping> RobotDiscreteActionTextSerializer::loadActionMapping(abt::BeliefNode* owner, std::istream& is)
{    
    std::unique_ptr<oppt::RobotDiscreteActionMapping> discMap = std::make_unique<oppt::RobotDiscreteActionMapping>(
                owner,
                static_cast<abt::DiscretizedActionPool*>(getSolver()->getActionPool()),
                std::vector<long> { });
    loadActionMapping(*discMap, is);
    return std::move(discMap);
}

void RobotDiscreteActionTextSerializer::loadActionMapping(abt::DiscretizedActionMap& discMap, std::istream& is)
{
    oppt::RobotDiscreteActionMapping *robotDiscreteActionMapping = static_cast<oppt::RobotDiscreteActionMapping *>(&discMap);
    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream sstr4(line);
    long numVisits, nChildren, totalVisitCount;
    sstr4 >> numVisits >> tmpStr >> tmpStr >> tmpStr;
    for (size_t i = 0; i != numVisits; ++i) {
	robotDiscreteActionMapping->increaseNumVisitedEntries();
    }
    
    sstr4 >> nChildren >> tmpStr;
    for (size_t i = 0; i != nChildren; ++i) {
	robotDiscreteActionMapping->increaseNChildren();
    }
    
    //sstr4 >> discMap.nChildren_ >> tmpStr;
    sstr4 >> totalVisitCount;
    robotDiscreteActionMapping->increaseVisitCount(totalVisitCount);

    std::getline(is, line);
    std::istringstream sstr(line);
    std::getline(sstr, tmpStr, '(');
    std::getline(sstr, tmpStr, ')');
    if (tmpStr != "") {
        std::istringstream sstr2(tmpStr);
        std::string actionString;
        while (!sstr2.eof()) {
            std::getline(sstr2, actionString, ',');
            long code;
            std::istringstream(actionString) >> code;
	    robotDiscreteActionMapping->addToBinSequence(code);            
        }
    }

    for (long i = 0; i < robotDiscreteActionMapping->getNumVisitedEntries() ; i++) {
        // The first line contains info from the mapping.
        std::getline(is, line);
        std::istringstream sstr2(line);
        long binNumber;
        sstr2 >> tmpStr >> binNumber;
        std::getline(sstr2, tmpStr, '(');
        std::getline(sstr2, tmpStr, ')');
        std::getline(sstr2, tmpStr, ':');
        FloatType meanQValue, totalQValue;
        long visitCount;
        std::string legalString;
        sstr2 >> meanQValue >> tmpStr;
        sstr2 >> visitCount >> tmpStr >> tmpStr;
        sstr2 >> totalQValue >> legalString;

        bool hasChild = true;
        std::string tmpStr1, tmpStr2;
        sstr2 >> tmpStr1 >> tmpStr2;
        if (tmpStr1 == "NO" && tmpStr2 == "CHILD") {
            hasChild = false;
        }

        // Create an entry to hold the action node.
        RobotDiscretizedActionMapEntry &entry = robotDiscreteActionMapping->getEntries()[binNumber];
        //DiscretizedActionMapEntry& entry = discMap.entries_[binNumber];
	entry.setBinNumber(binNumber);
        //entry.binNumber_ = binNumber;
	entry.setActionMapping(robotDiscreteActionMapping);
        //entry.map_ = &discMap;
	entry.setMeanQValue(meanQValue);
        //entry.meanQValue_ = meanQValue;
	entry.setVisitCount(visitCount);
        //entry.visitCount_ = visitCount;
	entry.setTotalQValue(totalQValue);
        //entry.totalQValue_ = totalQValue;
	entry.setLegal(legalString != "ILLEGAL");
        //entry.isLegal_ = (legalString != "ILLEGAL");

        // Read in the action node itself.
        if (hasChild) {
            entry.childNode_ = std::make_unique<abt::ActionNode>(&entry);
            abt::ActionNode* node = entry.childNode_.get();
            load(*node, is);
        }
    }
    
    // Any bins we are supposed to try must be considered legal.
    for (long binNumber : *(robotDiscreteActionMapping->getBinSequence())) {
        robotDiscreteActionMapping->getEntries()[binNumber].isLegal_ = true;
    }

}

RobotContinuousActionTextSerializer::RobotContinuousActionTextSerializer(abt::Solver* solver):
    RobotSerializer(solver)
{

}

void RobotContinuousActionTextSerializer::saveAction(abt::Action const* action, std::ostream& os)
{
    if (!action) {
        os << "NULL";
        return;
    }

    static_cast<const robot::ContinuousRobotAction*>(action)->serialize(os);

}

std::unique_ptr<abt::Action> RobotContinuousActionTextSerializer::loadAction(std::istream& is)
{
    oppt::Serializer* serializer =
        static_cast<robot::RobotModel*>(getSolver()->getModel())->getRobotEnvironment()->getRobot()->getSerializer();
    oppt::ActionSharedPtr robotAction = serializer->as<oppt::RobotObjectSerializer>()->loadAction(is);
    if (!robotAction)
	return nullptr;
    return std::make_unique<robot::ContinuousRobotAction>(robotAction);
}

void RobotContinuousActionTextSerializer::saveConstructionData(const ThisActionConstructionDataBase* baseData, std::ostream& os)
{

}

std::unique_ptr<RobotContinuousActionTextSerializer::ThisActionConstructionDataBase>
RobotContinuousActionTextSerializer::loadConstructionData(std::istream& is)
{
    return nullptr;
}

}
