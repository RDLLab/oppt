#include "HistorySequence.hpp"
#include "HistoryEntry.hpp"

namespace abt
{
OPPTHistorySequence::OPPTHistorySequence():
    HistorySequence()
{

}

OPPTHistorySequence::OPPTHistorySequence(long int id):
    HistorySequence(id)
{

}

abt::HistoryEntry* OPPTHistorySequence::addEntry()
{
    std::unique_ptr<abt::HistoryEntry> newEntry = std::make_unique<abt::OPPTHistoryEntry>(
            this, entrySequence_.size());    
    abt::HistoryEntry *newEntryReturn = newEntry.get();
    entrySequence_.push_back(std::move(newEntry));
    return newEntryReturn;
}


}
