#include "TreeInspector.hpp"
#include "solvers/ABT/robotModel/belief/BeliefNode.hpp"
#include "solvers/ABT/solver/StatePool.hpp"

using std::cout;
using std::endl;

namespace oppt
{
TreeInspector::TreeInspector()
{

}

void TreeInspector::makeTreeStatistics(const abt::Simulator* simulator, std::ofstream& os) const
{
    long tTreeDepth = totalTreeDepth(simulator);
    long rTreeDepth = relativeTreeDepth(simulator);
    long nNodes = numNodes(simulator);
    abt::BeliefNode* currentBelief = simulator->getAgent()->getCurrentBelief();
    long nNodesSubtree = 0;
    long totalNumDynamics = 0;
    numNodesBelief(currentBelief, nNodesSubtree, totalNumDynamics);
    os << "TotalTreeDepth: " << tTreeDepth << endl;
    os << "RelativeTreeDepth: " << rTreeDepth << endl;
    os << "TotalNumNodesInTree: " << nNodes << endl;
    os << "NumNodesSubtree: " << nNodesSubtree << endl;
    os << "TotalNumPropagations: " << totalNumDynamics << endl;
    //cout << "Total Tree Depth: " << tTreeDepth << endl;
    //cout << "Relative Tree Depth: " << rTreeDepth << endl;
    PRINT("Nodes In Tree: " + std::to_string(nNodes));
    PRINT("Nodes After Current Belief: " + std::to_string(nNodesSubtree));
    //cout << "Total num dynamics: " << totalNumDynamics << endl;
    //cout << "Num approximate dynamics: " << numApproximateDynamics << endl;

    /**auto actionMappingEntries = currentBelief->getMapping()->getChildEntries();
    cout << "NUM ACTION MAPPING ENTRIES: " << actionMappingEntries.size() << endl;
    for (auto &actionMappingEntry: actionMappingEntries) {
    cout << "ACTION: " << *(actionMappingEntry->getAction().get()) << endl;
    auto childObservationMappingEntries = actionMappingEntry->getActionNode()->getMapping()->getChildEntries();
    for (auto & childObservationEntry : childObservationMappingEntries) {
        cout << "O: " << *(childObservationEntry->getObservation().get()) << endl;
        cout << "visitCount: " << childObservationEntry->getVisitCount() << endl;
    }
    cout << "NUM OBSERVATION ENTRIES: " <<  actionMappingEntry->getActionNode()->getMapping()->getChildEntries().size() << endl;
    }*/

    //cout << "NUM CHILD OBSERVATION: " << childObservationMappingEntries.size() << endl;
}

long int TreeInspector::getTreeDepthRecursive(abt::BeliefNode* belief, long currentDepth) const
{
    long depth = currentDepth + 1;
    long maxDepth = depth;
    auto actionMapping = belief->getMapping();
    auto actionMappingEntries = actionMapping->getChildEntries();
    for (auto & actionMappingEntry : actionMappingEntries) {
        auto childObservationMappingEntries = actionMappingEntry->getActionNode()->getMapping()->getChildEntries();
        for (auto & childObservationEntry : childObservationMappingEntries) {
            auto beliefNode = childObservationEntry->getBeliefNode();
            long childDepth = getTreeDepthRecursive(beliefNode, depth);
            if (childDepth > maxDepth) {
                maxDepth = childDepth;
            }
        }
    }

    return maxDepth;
}

void TreeInspector::getTotalNumberOfStates(const abt::Simulator* simulator) const
{
    auto statePool = simulator->getSolver()->getStatePool();
    auto beliefNodes = simulator->getSolver()->getPolicy()->getNodes();
    long numStatesInTree = 0;
    std::vector<abt::State const*> allStatesInTree;
    std::vector<abt::State const*> allStatesInPool = statePool->getStates();
    for (auto & node : beliefNodes) {
        auto beliefStates = node->getStates();
        allStatesInTree.insert(allStatesInTree.end(), beliefStates.begin(), beliefStates.end());
        numStatesInTree += node->getNumberOfParticles();
    }

    long numLostStates = 0;
    for (auto & stateInPool : allStatesInPool) {
        if (std::find(allStatesInTree.begin(), allStatesInTree.end(), stateInPool) == allStatesInTree.end())
            numLostStates++;
    }

    long numStatesInPool = statePool->getNumberOfStates();
    cout << "num states in tree: " << numStatesInTree << endl;
    cout << "num states in pool: " << numStatesInPool << endl;
    cout << "numLostStates: " << numLostStates << endl;
}

long int TreeInspector::totalTreeDepth(const abt::Simulator* simulator) const
{
    long maxDepth = 0;
    for (auto & beliefNode : simulator->getSolver()->getPolicy()->getNodes()) {
        long depth = beliefNode->getDepth();
        if (depth > maxDepth) {
            maxDepth = depth;
        }
    }

    return maxDepth;
}

long int TreeInspector::relativeTreeDepth(const abt::Simulator* simulator) const
{
    long maxDepth = 0;
    for (auto & beliefNode : simulator->getSolver()->getPolicy()->getNodes()) {
        long depth = beliefNode->getDepth() - simulator->getAgent()->getCurrentBelief()->getDepth();
        if (depth > maxDepth) {
            maxDepth = depth;
        }
    }

    return maxDepth;
}

long int TreeInspector::numNodes(const abt::Simulator* simulator) const
{
    long numNodes = simulator->getSolver()->getPolicy()->getNodes().size();
    return numNodes;
}

void TreeInspector::numNodesBelief(abt::BeliefNode* belief, long& currentNum, long& totalNumDynamics) const
{
    currentNum += 1;
    totalNumDynamics += static_cast<oppt::BeliefNode*>(belief)->getTotalNumDynamicsUsed();
    auto actionMapping = belief->getMapping();
    auto actionMappingEntries = actionMapping->getChildEntries();
    for (auto & actionMappingEntry : actionMappingEntries) {
        auto childObservationMappingEntries = actionMappingEntry->getActionNode()->getMapping()->getChildEntries();
        for (auto & childObservationEntry : childObservationMappingEntries) {
            auto beliefNode = childObservationEntry->getBeliefNode();
            numNodesBelief(beliefNode, currentNum, totalNumDynamics);
        }
    }

}

}
