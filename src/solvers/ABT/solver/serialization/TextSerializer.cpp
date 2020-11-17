/** @file TextSerializer.cpp
 *
 * Contains the implementation of the TextSerializer class, which provides a human-readable
 * serialization of all of the basic data structures.
 */
#include "TextSerializer.hpp"

#include <cstddef>                      // for size_t

#include <iomanip>
#include <iostream>
#include <map>                          // for _Rb_tree_iterator, map<>::mapped_type
#include <sstream>                      // for stringstream
#include <string>                       // for operator>>, getline, string, operator<<
#include <unordered_set>                // for unordered_set<>::iterator, _Node_iterator
#include <utility>                      // for move, pair
#include <vector>                       // for vector

#include "solvers/ABT/solver/ActionNode.hpp"               // for ActionNode
#include "solvers/ABT/solver/BeliefNode.hpp"               // for BeliefNode, BeliefNode::ActionMap, BeliefNode::currId
#include "solvers/ABT/solver/BeliefTree.hpp"               // for BeliefTree
#include "solvers/ABT/solver/Histories.hpp"                // for Histories
#include "solvers/ABT/solver/HistoryEntry.hpp"             // for HistoryEntry
#include "solvers/ABT/solver/HistorySequence.hpp"          // for HistorySequence
#include "solvers/ABT/solver/Solver.hpp"                   // for Solver
#include "solvers/ABT/solver/StateInfo.hpp"                // for StateInfo, StateInfo::currId
#include "solvers/ABT/solver/StatePool.hpp"                // for StatePool, StatePool::StateInfoSet

#include "solvers/ABT/solver/abstract-problem/Action.hpp"              // for Action
#include "solvers/ABT/solver/abstract-problem/ModelChange.hpp"              // for ModelChange
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "solvers/ABT/solver/abstract-problem/State.hpp"                    // for State, operator<<

#include "solvers/ABT/solver/belief-estimators/estimators.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationMapping.hpp"          // for ObservationMapping

#include "Serializer.hpp"               // for Serializer

namespace abt {
/* ------------------ Saving change sequences -------------------- */
void TextSerializer::saveChangeSequence(ChangeSequence const &sequence, std::ostream &os) {
    for (auto &entry : sequence) {
        os << "t " << entry.first << " : " << entry.second.size() << std::endl;
        for (std::unique_ptr<ModelChange> const &change : entry.second) {
            saveModelChange(*change, os);
            os << std::endl;
        }
    }
}

ChangeSequence TextSerializer::loadChangeSequence(
        std::istream &is){
    std::map<long, std::vector<std::unique_ptr<ModelChange>>> changes;
    std::string line;
    while (std::getline(is, line)) {
        oppt::trim(line);
        if (line.empty()) {
            continue;
        }

        std::string tmpStr;
        long time;
        long nChanges;
        std::istringstream(line) >> tmpStr >> time >> tmpStr >> nChanges;

        changes[time] = std::vector<std::unique_ptr<ModelChange>>();
        for (int i = 0; i < nChanges; i++) {
            std::getline(is, line);
            std::istringstream sstr(line);
            changes[time].push_back(loadModelChange(sstr));
        }
    }
    return changes;
}

void TextSerializer::saveModelChange(ModelChange const &/*change*/, std::ostream &/*os*/) {
    // Do nothing!
}
std::unique_ptr<ModelChange> TextSerializer::loadModelChange(std::istream &/*is*/) {
    return nullptr;
}

/* ------------------ Saving transition parameters -------------------- */
void TextSerializer::saveTransitionParameters(
        TransitionParameters const */*tp*/, std::ostream &/*os*/) {
    // Do nothing!
}
std::unique_ptr<TransitionParameters> TextSerializer::loadTransitionParameters(
        std::istream &/*is*/) {
    return nullptr;
}

/* ------------------ Saving historical data -------------------- */
void TextSerializer::saveHistoricalData(HistoricalData const */*data*/,
        std::ostream &/*os*/) {
    // Do nothing!
}
std::unique_ptr<HistoricalData> TextSerializer::loadHistoricalData(
        std::istream &/*is*/) {
    return nullptr;
}

/* ------------------ Saving the state pool -------------------- */
void TextSerializer::save(StateInfo const &info, std::ostream &os) {
    os << "State ";
    os << std::setw(6) << info.id_;
    os << " : ";
    saveState(info.state_.get(), os);
}

void TextSerializer::load(StateInfo &info, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> info.id_ >> tmpStr;    
    info.state_ = loadState(is);
}

void TextSerializer::save(StatePool const &pool, std::ostream &os) {
    os << "STATESPOOL-BEGIN" << std::endl;
    os << "numStates: " << pool.stateInfoMap_.size() << std::endl;
    for (std::unique_ptr<StateInfo> const &stateInfo : pool.statesByIndex_) {
        save(*stateInfo, os);
        os << std::endl;
    }
    os << "STATESPOOL-END" << std::endl;
}

void TextSerializer::load(StatePool &pool, std::istream &is) {
    std::string line;
    std::getline(is, line);
    while (line.find("STATESPOOL-BEGIN") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }
    std::getline(is, line);
    std::string tmpStr;
    long nStates;
    std::istringstream(line) >> tmpStr >> nStates;

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        std::unique_ptr<StateInfo> newStateInfo(std::make_unique<StateInfo>());
        std::istringstream sstr(line);	
        load(*newStateInfo, sstr);
        pool.add(std::move(newStateInfo));
        std::getline(is, line);	
    }
}

/* ------------------ Saving the histories -------------------- */
void TextSerializer::save(HistoryEntry const &entry, std::ostream &os) {
    os << "HistoryEntry < ";
    os << entry.owningSequence_->id_ << " " << entry.entryId_ << " >: (S";
    oppt::print_with_width(entry.stateInfo_->id_, os, 6,
            std::ios_base::left);

    os << " ";
    std::stringstream sstr;
    saveAction(entry.action_.get(), sstr);
    oppt::print_with_width(sstr.str(), os,
            getActionColumnWidth(),
            std::ios_base::left);

    os << " ";
    sstr.clear();
    sstr.str("");
    saveTransitionParameters(entry.transitionParameters_.get(), sstr);
    oppt::print_with_width(sstr.str(), os,
            getTPColumnWidth(),
            std::ios_base::left);

    os << " ";
    sstr.clear();
    sstr.str("");
    saveObservation(entry.observation_.get(), sstr);
    oppt::print_with_width(sstr.str(), os,
            getObservationColumnWidth(),
            std::ios_base::left);

    os << " r:";
    oppt::print_double(entry.immediateReward_, os, 6, 2,
            std::ios_base::fixed | std::ios_base::showpos
                    | std::ios_base::left);

    os << ");   S:";
    saveState(entry.stateInfo_->getState(), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    long seqId;
    is >> tmpStr >> tmpStr >> seqId >> entry.entryId_;
    std::getline(is, tmpStr, 'S');
    is >> stateId;
    entry.action_ = std::move(loadAction(is));
    entry.transitionParameters_ = std::move(loadTransitionParameters(is));
    entry.observation_ = std::move(loadObservation(is));
    std::getline(is, tmpStr, ':');
    is >> entry.immediateReward_;
    entry.registerState(getSolver()->getStatePool()->getInfoById(stateId));
}

void TextSerializer::save(HistorySequence const &seq, std::ostream &os) {
    os << "HistorySequence " << seq.id_;
    os << " - length " << seq.entrySequence_.size() << std::endl;
    for (std::unique_ptr<HistoryEntry> const &entry : seq.entrySequence_) {
        save(*entry, os);
        os << std::endl;
    }
}

void TextSerializer::load(HistorySequence &seq, std::istream &is) {
    long seqLength;
    std::string line;
    std::getline(is, line);
    std::istringstream sstr(line);
    std::string tmpStr;
    sstr >> tmpStr >> seq.id_ >> tmpStr >> tmpStr >> seqLength;
    for (int i = 0; i < seqLength; i++) {
        std::getline(is, line);
        std::unique_ptr<HistoryEntry> entry(std::make_unique<HistoryEntry>());
        sstr.clear();
        sstr.str(line);
        load(*entry, sstr);
        entry->owningSequence_ = &seq;
        seq.entrySequence_.push_back(std::move(entry));
    }
}

void TextSerializer::save(Histories const &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << std::endl;
    os << "numHistories: " << histories.getNumberOfSequences() << std::endl;
    for (std::unique_ptr<HistorySequence> const &seq : histories.sequencesById_) {
        save(*seq, os);
    }
    os << "HISTORIES-END" << std::endl;
}

void TextSerializer::load(Histories &histories, std::istream &is) {
    std::string line;
    std::getline(is, line);
    while (line.find("HISTORIES-BEGIN") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }

    long numHistories;
    std::string tmpStr;
    is >> tmpStr >> numHistories;
    std::getline(is, line);

    for (int i = 0; i < numHistories; i++) {
        std::unique_ptr<HistorySequence> sequence(std::make_unique<HistorySequence>());
        load(*sequence, is);
        histories.sequencesById_.push_back(std::move(sequence));
    }
    std::getline(is, line);
    while (line.find("HISTORIES-END") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }
}


/* ------------------ Saving the policy tree -------------------- */
void TextSerializer::save(ActionNode const &node, std::ostream &os) {
    saveObservationMapping(*node.getMapping(), os);
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    node.setMapping(loadObservationMapping(&node, is));
}

void TextSerializer::save(BeliefNode const &node, std::ostream &os) {
    if (node.getNumberOfParticles() == 0) {
        os << "No particles!" << std::endl;
    } else {
        os << node.getNumberOfParticles() << " particles begin" << std::endl;
        int count = 0;
        for (auto it = node.particles_.begin(); it != node.particles_.end();
                ++it) {
            HistoryEntry const *entry = *it;
            os << "( ";
            os << entry->owningSequence_->id_;
            os << " ";
            os << entry->entryId_ << " )";
            count += 1;
            if (count >= NUM_PARTICLES_PER_LINE
                    || (it+1) == node.particles_.end()) {
                os << std::endl;
                count = 0;
            } else {
                os << ", ";
            }
        }
        os << "particles end" << std::endl;
    }
    
    saveHistoricalData(node.getHistoricalData(), os);    
    saveActionMapping(*node.getMapping(), os);
}

void TextSerializer::load(BeliefNode &node, std::istream &is) {
    std::string line;
    std::getline(is, line);
    if (line != "No particles!") {
        long nParticles;
        std::string tmpStr;
        std::istringstream(line) >> nParticles;

        std::getline(is, line);
        long numParticlesRead = 0;
        while (line != "particles end") {
            std::istringstream sstr(line);
            for (int i = 0; i < NUM_PARTICLES_PER_LINE; i++) {
                long seqId, entryId;
                sstr >> tmpStr >> seqId >> entryId >> tmpStr;
                HistorySequence *sequence = getSolver()->histories_->getSequence(
                        seqId);
                HistoryEntry *entry = sequence->getEntry(entryId);
                entry->registerNode(&node);
                numParticlesRead++;
                if (numParticlesRead == nParticles) {
                    break;
                }
            }
            std::getline(is, line);
        }
    }
    node.setHistoricalData(loadHistoricalData(is));
    node.setMapping(loadActionMapping(&node, is));
    getSolver()->getEstimationStrategy()->setValueEstimator(getSolver(), &node);
}

void TextSerializer::save(BeliefTree const &tree, std::ostream &os) {
    os << "BELIEFTREE-BEGIN" << std::endl;
    os << "numNodes: " << tree.getNumberOfNodes() << std::endl;
    for (BeliefNode *node : tree.allNodes_) {
        os << "NODE " << node->id_ << std::endl;
        save(*node, os);
        os << std::endl;
    }
    os << "BELIEFTREE-END" << std::endl;
}

void TextSerializer::load(BeliefTree &tree, std::istream &is) {
    std::string line;
    std::getline(is, line);
    while (line.find("BELIEFTREE-BEGIN") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }
    std::getline(is, line);

    long nNodes;
    std::string tmpStr;
    std::istringstream(line) >> tmpStr >> nNodes;    
    tree.allNodes_.resize(nNodes);    

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        long nodeId;
        std::istringstream(line) >> tmpStr >> nodeId;	
        BeliefNode *node = tree.getNode(nodeId);
        load(*node, is);
        node->recalculateValue();
        // Ignore an empty line after each belief node.
        std::getline(is, line);
        std::getline(is, line);
    }
}

int TextSerializer::getActionColumnWidth(){
    return 0;
}
int TextSerializer::getTPColumnWidth() {
    return 0;
}
int TextSerializer::getObservationColumnWidth() {
    return 0;
}
} /* namespace abt */
