#include "BeliefTree.hpp"

namespace oppt
{
BeliefTree::BeliefTree(abt::Solver* solver):
    abt::BeliefTree(solver),
    counter_(-1)
{

}

abt::BeliefNode* BeliefTree::reset()
{
    root_ = nullptr;
    root_ = std::make_unique<oppt::BeliefNode>(solver_);
    abt::BeliefNode* rootPtr = static_cast<abt::BeliefNode*>(root_.get());
    counter_ = -1;
    return rootPtr;
}

void BeliefTree::addNode(BeliefNode* node)
{    
    long id = node->id_;
    if (id < 0) {
        // Negative ID => add it to the back of the vector.
	counter_++;
        id = counter_;
        node->id_ = id;
        allNodes_.push_back(nullptr);
    }
    if (allNodes_[id] != nullptr) {
        debug::show_message("ERROR: Node already exists - overwriting!!");
    }
    allNodes_[counter_] = node;

}

/** Removes the given node from the index of nodes. */
void BeliefTree::removeNode(BeliefNode* node)
{
    /**long id = node->id_;

    long lastNodeId = allNodes_.size() - 1;

    if (id < 0 || id > lastNodeId) {
        debug::show_message("ERROR: Node ID is out of bounds.");	
        return;
    }
    
    if (allNodes_[id] != node) {
        debug::show_message("ERROR: Node ID does not match index.");
        return;
    }

    // Now remove the node from the index.
    if (id < lastNodeId) {
        abt::BeliefNode *lastNode = allNodes_[lastNodeId];
        //lastNode->id_ = id;
        allNodes_[id] = lastNode;
    }
    allNodes_.pop_back();*/
    allNodes_.erase(std::remove(allNodes_.begin(), allNodes_.end(), node), allNodes_.end());
}

}



