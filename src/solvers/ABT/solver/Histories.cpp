/** @file Histories.cpp
 *
 * Contains the implementation of the Histories class
 */
#include "Histories.hpp"

#include <utility>                      // for move

#include "oppt/global.hpp"                     // for make_unique

#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence

namespace abt {
Histories::Histories() :
        sequencesById_() {
}

/* ------------------- Retrieving sequences ------------------- */
long Histories::getNumberOfSequences() const {
    return sequencesById_.size();
}
HistorySequence *Histories::getSequence(long seqId) const {
    return sequencesById_[seqId].get();
}


/* ============================ PRIVATE ============================ */


/* ---------------- Adding / removing sequences  ---------------- */
void Histories::reset() {
    sequencesById_.clear();
}
HistorySequence *Histories::createSequence() {
    std::unique_ptr<HistorySequence> histSeq(
            std::make_unique<HistorySequence>(sequencesById_.size()));
    HistorySequence *rawPtr = histSeq.get();
    sequencesById_.push_back(std::move(histSeq));
    return rawPtr;
}
void Histories::deleteSequence(HistorySequence *sequence) {
    // Retrieve the current ID of the sequence, which should be its position in the vector.
    long seqId = sequence->id_;

    if (sequencesById_[seqId].get() != sequence) {
        debug::show_message("ERROR: sequence ID does not match its index!");
    }

    // Deregister and clear the sequence.
    sequence->erase();
    if (seqId < static_cast<long>(sequencesById_.size()) - 1) {
        sequencesById_[seqId] = std::move(sequencesById_[sequencesById_.size()-1]);
        sequencesById_[seqId]->id_ = seqId;
    }
    sequencesById_.pop_back();
    /**if (sequence)
	sequence->erase();
    auto pre = [sequence](const std::unique_ptr<HistorySequence> &hs){
	return hs.get() == sequence;
    };
    sequencesById_.erase(std::remove_if(sequencesById_.begin(), sequencesById_.end(), pre), sequencesById_.end());*/
}
} /* namespace abt */
