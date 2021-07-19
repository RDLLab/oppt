#include "RobotSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include <fstream>

#include <sstream>

#include "RobotObservation.hpp"
#include "RobotState.hpp"
#include "belief/BeliefNode.hpp"

namespace shared
{


std::vector<std::string> split(const std::string& s, char delim)
{
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> elems;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

void saveVector(std::vector<long> values, std::ostream& os)
{
    os << "(";
    for (auto it = values.begin(); it != values.end(); it++) {
        os << *it;
        if ((it + 1) != values.end()) {
            os << ", ";
        }
    }
    os << ")";
}

template <typename valueType>
std::vector<valueType> loadVector(std::istream& is)
{
    std::vector<valueType> values;
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');    
    std::istringstream sstr(tmpStr);
    while (std::getline(sstr, tmpStr, ' ')) {
        valueType value;
        std::istringstream(tmpStr) >> value;
        values.push_back(value);
    }
    return values;
}

template <typename valueType>
std::vector<valueType> loadVectorFromString(std::string& str)
{
    std::vector<valueType> values;
    str.erase(std::remove(str.begin(), str.end(), '('), str.end());
    str.erase(std::remove(str.begin(), str.end(), ')'), str.end());
    std::istringstream sstr(str);
    while (std::getline(sstr, str, ' ')) {
	valueType value;
        std::istringstream(str) >> value;
        values.push_back(value);
    }
   
    return values;
}

template<typename valueType>
std::vector<std::vector<valueType>> load2Vector(std::istream& is)
{
    std::vector<std::vector<valueType>> values;
    std::string tmpStr;
    std::getline(is, tmpStr, '|');
    std::getline(is, tmpStr, '|');    
    std::vector<std::string> coords = split(tmpStr, ',');
    for (std::string &k: coords) {
	std::vector<valueType> vertice = loadVectorFromString<valueType>(k);
	values.push_back(vertice);
    }
   
    return values;
}

RobotSerializer::RobotSerializer(abt::Solver* solver) :
    abt::TextSerializer()
{
}

void RobotSerializer::saveState(abt::State const* state, std::ostream& os)
{
    if (state == nullptr) {
        os << "NULL";
        return;
    }
    shared::RobotState const& robotState = static_cast<shared::RobotState const&>(*state);
    robotState.serialize(os);
}

std::unique_ptr<abt::State> RobotSerializer::loadState(std::istream& is)
{
    oppt::Serializer* serializer =
        static_cast<robot::RobotModel*>(getSolver()->getModel())->getRobotEnvironment()->getRobot()->getSerializer();
    if (!serializer)
	ERROR("Serializer is null");
    oppt::RobotStateSharedPtr robotState = serializer->as<oppt::RobotObjectSerializer>()->loadState(is);
    return std::make_unique<shared::RobotState>(robotState);
}

void RobotSerializer::saveObservation(abt::Observation const* obs, std::ostream& os)
{
    if (obs == nullptr) {
        os << "NULL";
        return;
    }
    static_cast<const shared::RobotObservation *>(obs)->serialize(os);
    //shared::RobotObservation const& o = static_cast<shared::RobotObservation const&>(*obs);
    //o.serialize(os);
}

std::unique_ptr<abt::Observation> RobotSerializer::loadObservation(std::istream& is)
{
    oppt::Serializer* serializer =
        static_cast<robot::RobotModel*>(getSolver()->getModel())->getRobotEnvironment()->getRobot()->getSerializer();
    oppt::ObservationSharedPtr observation = serializer->as<oppt::RobotObjectSerializer>()->loadObservation(is);
    if (!observation)
	return nullptr;
    return std::make_unique<shared::RobotObservation>(observation);
}

/* ------------------ Saving change sequences -------------------- */
void RobotSerializer::saveModelChange(abt::ModelChange const& change, std::ostream& os)
{
    /**EnvironmentChange const& environmentChange = static_cast<EnvironmentChange const&>(change);
    if (environmentChange.environmentType == "simple") {
        SimpleEnvironmentChange const& simpleEnvironmentChange =
            static_cast<SimpleEnvironmentChange const&>(environmentChange);
        os << environmentChange.changeType;
        os << " ";
        os << simpleEnvironmentChange.bodyName;
        os << ": (";
        for (size_t i = 0; i < simpleEnvironmentChange.newDims.size() - 1; i++) {
            os << simpleEnvironmentChange.newDims[i] << " ";
        }
        os << simpleEnvironmentChange.newDims[simpleEnvironmentChange.newDims.size() - 1] << ") ";
        if (simpleEnvironmentChange.traversable) {
            os << "true ";
        } else {
            os << "false ";
        }

        if (simpleEnvironmentChange.observable) {
            os << "true ";
        } else {
            os << "false ";
        }

        os << simpleEnvironmentChange.traversalCost;
    }*/
}

std::unique_ptr<abt::ModelChange> RobotSerializer::loadModelChange(std::istream& is)
{
    //TODO IMPLEMENT ME!
    ERROR("Loading changes is currently not implemented");
    return nullptr;    
}

/*--------------- RobotApproximateObservationTextSerializer -----------------------*/
std::unique_ptr<abt::ObservationMapping> RobotSerializer::loadObservationMapping(abt::ActionNode* owner, std::istream& is)
{
    std::unique_ptr<abt::ObservationMapping> map(
            getSolver()->getObservationPool()->createObservationMapping(owner));
    oppt::ApproximateABTObservationMap &approxMap =
            (static_cast<oppt::ApproximateABTObservationMap &>(*map));
    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream totalsStream(line);
    long nChildren, totalVisitCount;
    totalsStream >> nChildren >> tmpStr >> tmpStr;
    totalsStream >> totalVisitCount;    
    approxMap.setTotalVisitCount(totalVisitCount);

    for (int i = 0; i < nChildren; i++) {
        std::getline(is, line);
        std::istringstream entryStream(line);
        std::unique_ptr<abt::Observation> obs = loadObservation(entryStream);

        entryStream >> tmpStr >> tmpStr;
        std::getline(entryStream, tmpStr, ';');
        long childId;
        std::istringstream(tmpStr) >> childId;
        long visitCount;
        entryStream >> visitCount;

        // Create the entry with appropriate values.
        std::unique_ptr<abt::ApproximateObservationMapEntry> entry = (
                std::make_unique<oppt::ApproximateABTObservationMapEntry>());
	oppt::ApproximateABTObservationMapEntry *fEntry = static_cast<oppt::ApproximateABTObservationMapEntry *>(entry.get());
	fEntry->setMapping(&approxMap);
        //entry->map_ = &approxMap;
	fEntry->setObservation(std::move(obs));
        //entry->observation_ = std::move(obs);
	fEntry->setVisitCount(visitCount);
        //entry->visitCount_ = visitCount;
	auto child = std::make_unique<oppt::BeliefNode>(childId, entry.get(), getSolver());
	fEntry->setChildNode(std::move(child));
        //entry->childNode_ = std::make_unique<BeliefNode>(childId, entry.get(), getSolver());

        // Add the entry to the vector.
	approxMap.addToEntries(std::move(entry));
        //approxMap.entries_.push_back(std::move(entry));
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}

}
