/** @file HistorySequence.cpp
 *
 * Contains the implementation of the HistorySequence class.
 */
#include "HistorySequence.hpp"

#include <limits>

#include <memory>                       // for unique_ptr
#include <utility>                      // for move
#include <vector>                       // for vector, __alloc_traits<>::value_type

#include "oppt/global.hpp"                     // for make_unique

#include "BeliefNode.hpp"
#include "BeliefTree.hpp"
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "StateInfo.hpp"                // for StateInfo

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation

#include "changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED
#include <iostream>

using std::cout;
using std::endl;

namespace abt {
HistorySequence::HistorySequence() :
    HistorySequence(-1) {
}

HistorySequence::HistorySequence(long id) :
    id_(id),
    entrySequence_(),
    startAffectedIdx_(std::numeric_limits<long>::max()),
    endAffectedIdx_(-1),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

// Do nothing!
HistorySequence::~HistorySequence() {
}

/* ------------------ Simple getters ------------------- */
long HistorySequence::getId() const {
    return id_;
}
long HistorySequence::getLength() const {
    return entrySequence_.size();
}
HistoryEntry *HistorySequence::getEntry(HistoryEntry::IdType entryId) const {    
    return entrySequence_[entryId].get();
}
HistoryEntry *HistorySequence::getFirstEntry() const {
    return entrySequence_.front().get();
}
HistoryEntry *HistorySequence::getLastEntry() const {
    return entrySequence_.back().get();
}
std::vector<State const *> HistorySequence::getStates() const {
    std::vector<State const *> states;
    for (std::unique_ptr<HistoryEntry> const &entry : entrySequence_) {
        states.push_back(entry->getState());
    }
    return states;
}


/* ============================ PRIVATE ============================ */


/* ----------- Methods to add or remove history entries ------------- */
void HistorySequence::erase(HistoryEntry::IdType firstEntryId) {
    for (auto it = entrySequence_.crbegin();
            it != entrySequence_.crend() && (*it)->entryId_ >= firstEntryId; it++) {
        (*it)->registerNode(nullptr);
        (*it)->registerState(nullptr);
    }
    entrySequence_.erase(entrySequence_.begin() + firstEntryId, entrySequence_.end());
}

HistoryEntry *HistorySequence::addEntry() {
    std::unique_ptr<HistoryEntry> newEntry = std::make_unique<HistoryEntry>(
            this, entrySequence_.size());    
    HistoryEntry *newEntryReturn = newEntry.get();
    entrySequence_.push_back(std::move(newEntry));
    return newEntryReturn;
}

/* -------------- Change flagging methods ---------------- */
void HistorySequence::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
    resetAffectedIndices();
}

void HistorySequence::setChangeFlags(HistoryEntry::IdType entryId, ChangeFlags flags) {
    setChangeFlags(flags);
    getEntry(entryId)->setChangeFlags(flags);
    addAffectedIndex(entryId);
}

// These are private and ought not to be called outside HistorySequence.
void HistorySequence::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}

void HistorySequence::resetAffectedIndices() {
    startAffectedIdx_ = std::numeric_limits<long>::max();
    endAffectedIdx_ = -1;
}

void HistorySequence::addAffectedIndex(HistoryEntry::IdType entryId) {
    if (startAffectedIdx_ > entryId) {
        startAffectedIdx_ = entryId;
    }
    if (endAffectedIdx_ < entryId) {
        endAffectedIdx_ = entryId;
    }
}
} /* namespace abt */
