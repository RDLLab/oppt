#include "Histories.hpp"
#include "HistorySequence.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace abt
{
OPPTHistories::OPPTHistories():
    Histories()
{
}

HistorySequence* OPPTHistories::createSequence()
{   
    std::unique_ptr<HistorySequence> histSeq(
            std::make_unique<OPPTHistorySequence>(sequencesById_.size()));
    HistorySequence *rawPtr = histSeq.get();
    sequencesById_.push_back(std::move(histSeq));
    return rawPtr;
}

}
