/** @file StateInfo.cpp
 *
 * Contains the implementation of the StateInfo class.
 */
#include "StateInfo.hpp"

#include <algorithm>                    // for find
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <utility>                      // for move
#include <vector>                       // for vector, vector<>::iterator

#include "abstract-problem/State.hpp"                    // for State
#include "changes/ChangeFlags.hpp"              // for ChangeFlags, ChangeFlags::UNCHANGED
#include <iostream>

using std::cout;
using std::endl;

namespace abt
{
class BeliefNode;
class HistoryEntry;

StateInfo::StateInfo(std::unique_ptr<State> state) :
    state_(std::move(state)),
    id_(-1),
    usedInHistoryEntries_(),
    changeFlags_(ChangeFlags::UNCHANGED)
{
}

// Constructor for serialization.
StateInfo::StateInfo() :
    StateInfo(nullptr)
{
}

// Do nothing!
StateInfo::~StateInfo()
{
}


/* ---------------------- Simple getters  ---------------------- */
long StateInfo::getId() const
{
    return id_;
}
State const* StateInfo::getState() const
{
    return state_.get();
}


/* ============================ PRIVATE ============================ */


/* ----------------- History entry registration  ----------------- */
void StateInfo::addHistoryEntry(HistoryEntry* entry)
{
    usedInHistoryEntries_.insert(entry);
}
void StateInfo::removeHistoryEntry(HistoryEntry* entry)
{
    usedInHistoryEntries_.erase(entry);
}

/* ---------------------- Model change handling  ---------------------- */
void StateInfo::resetChangeFlags()
{
    changeFlags_ = ChangeFlags::UNCHANGED;
}

void StateInfo::setChangeFlags(ChangeFlags flags)
{
    changeFlags_ |= flags;
}

void StateInfo::setTerminal(const bool& terminal)
{
    isTerminal_ = terminal;
}

bool StateInfo::isTerminal() const
{
    return isTerminal_;
}

} /* namespace abt */
