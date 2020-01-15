#include "HistoryEntry.hpp"
#include "solvers/ABT/robotModel/belief/BeliefNode.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace abt
{
OPPTHistoryEntry::OPPTHistoryEntry(abt::HistorySequence* owningSequence, IdType entryId):
    abt::HistoryEntry(owningSequence, entryId)
{

}

OPPTHistoryEntry::OPPTHistoryEntry():
    abt::HistoryEntry()
{

}

void OPPTHistoryEntry::registerNode(abt::BeliefNode* node, bool addParticle)
{
    if (associatedBeliefNode_ == node) {
        return;
    }
    if (associatedBeliefNode_ != nullptr) {
        static_cast<oppt::BeliefNode*>(associatedBeliefNode_)->removeParticle(this);
	static_cast<oppt::BeliefNode*>(associatedBeliefNode_)->deregisterParticle(this);
        associatedBeliefNode_ = nullptr;
    }
    if (node != nullptr) {
        associatedBeliefNode_ = node;
        if (addParticle) {
            static_cast<oppt::BeliefNode*>(associatedBeliefNode_)->addParticle(this);
        }
        static_cast<oppt::BeliefNode*>(associatedBeliefNode_)->registerParticle(this);
    }
}


}
