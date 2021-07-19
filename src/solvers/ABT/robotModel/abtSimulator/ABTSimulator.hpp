#ifndef ___ABT_SIMULATOR_HPP__
#define ___ABT_SIMULATOR_HPP__
#include "solvers/ABT/solver/Simulator.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"
#include "solvers/ABT/solver/Solver.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/robotModel/treeInspector/TreeInspector.hpp"

using namespace oppt;

namespace abt
{
class ABTSimulator: public Simulator
{
public:
    ABTSimulator(std::unique_ptr<Model> model, Solver* solver, bool hasDynamicChanges);
    
    FloatType runSimulation(std::ofstream &os);
    
    bool stepSimulation(std::ofstream &os);
    
    TreeInspector *const getTreeInspector() const;
    
private:
    std::unique_ptr<TreeInspector> treeInspector_;
};
}

#endif
