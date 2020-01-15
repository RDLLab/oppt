/** @file BeliefTree.hpp
 *
 * Contains the BeliefTree class, which represents an entire belief tree.
 *
 * Most of the work is done in the individual classes for the mappings and nodes; this class
 * simply owns a root node, and keeps track of a vector of all of the nodes in the entire tree
 * for convenient iteration and serialization.
 */
#ifndef SOLVER_BELIEFTREE_HPP_
#define SOLVER_BELIEFTREE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"

namespace abt {
class ActionMapping;
class BasicSearchStrategy;
class BeliefNode;
class Solver;

/** Represents a belief tree, which is the core data structure representing a policy for the
 * ABT solver.
 *
 * This class just owns the root belief node; each individual node in the tree is owned by
 * its parent mapping entry.
 *
 * However, this belief tree also keeps track of an index of nodes, represented by a
 * vector of non-owning pointers to the individual belief nodes.
 * This allows for access by node ID, iteration and serialization.
 */
class BeliefTree {
    friend class Agent;
    friend class BeliefNode;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

public:
    /** Constructs an empty belief tree. */
    BeliefTree(Solver *solver);
    virtual ~BeliefTree();
    _NO_COPY_OR_MOVE(BeliefTree);

    /* ------------------- Simple getters --------------------- */
    /** Returns the root node. */
    BeliefNode *getRoot() const;
    /** Retrieves the node with the given ID. */
    BeliefNode *getNode(long id) const;
    /** Returns the number of belief nodes. */
    long getNumberOfNodes() const;
    /** Retrieves a vector of all belief nodes within the policy. */
    std::vector<BeliefNode *> getNodes() const;

protected:
    /* ------------------- Node index modification ------------------- */
    /** Adds the given node to the index of nodes. */
    virtual void addNode(BeliefNode *node);

    /** Removes the given node from the index of nodes. */
    virtual void removeNode(BeliefNode *node);

    /* ------------------- Tree modification ------------------- */
    /** Resets the tree, creating a new root node and returning it. */
    virtual BeliefNode *reset();

    /** Initializes the root node - for creating a new tree from scratch. */
    void initializeRoot();

    /** The ABT solver that owns this tree. */
    Solver *solver_;

    /** A vector of pointers to the all of the nodes of the tree. */
    std::vector<BeliefNode *> allNodes_;

    /** The root node for this tree. */
    std::unique_ptr<BeliefNode> root_;
};
} /* namespace abt */

#endif /* SOLVER_BELIEFTREE_HPP_ */
