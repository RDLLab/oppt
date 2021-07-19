#ifndef __TREE_INSPECTOR_HPP__
#define __TREE_INSPECTOR_HPP__
#include "solvers/ABT/solver/Simulator.hpp"
#include "oppt/opptCore/core.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/Solver.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"

namespace oppt
{
class TreeInspector
{
public:
    TreeInspector();
    
    void makeTreeStatistics(const abt::Simulator* simulator, std::ofstream& os) const;
    
    void getTotalNumberOfStates(const abt::Simulator* simulator) const;
    
private:
    long getTreeDepthRecursive(abt::BeliefNode* belief, long currentDepth) const;
    
    /**
     * Gets the depth of the belief tree
     */
    long totalTreeDepth(const abt::Simulator* simulator) const;
    
    /**
     * Gets the depth of the belief tree relative to the current belief
     */
    long relativeTreeDepth(const abt::Simulator* simulator) const;
    
    /**
     * Gets the total number of nodes in the belief tree
     */
    long numNodes(const abt::Simulator* simulator) const;
    
    /**
     * Gets the number of nodes in the subtree of a belief
     */
    void numNodesBelief(abt::BeliefNode* belief, long& currentNum, long &totalNumDynamics) const;

};

}

#endif
