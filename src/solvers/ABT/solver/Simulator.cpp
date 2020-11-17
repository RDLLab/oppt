/** @file Simulator.cpp
 *
 * Contains the implementation of the Simulator class.
 */
#include "Simulator.hpp"

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iomanip>
#include <iostream>                     // for operator<<, ostream, basic_ostream, endl, basic_ostream<>::__ostream_type, cout

#include "abstract-problem/Observation.hpp"
#include "abstract-problem/ModelChange.hpp"

#include "serialization/Serializer.hpp"

#include "Agent.hpp"
#include "BeliefNode.hpp"
#include "BeliefTree.hpp"
#include "HistoryEntry.hpp"
#include "HistorySequence.hpp"
#include "Solver.hpp"
#include "StatePool.hpp"
//#include "problems/robot_problem/RobotTransitionParameters.hpp"
#include "solvers/ABT/ABTOptions.hpp"

using std::cout;
using std::endl;

namespace abt
{
Simulator::Simulator(std::unique_ptr<Model> model, Solver* solver, bool hasDynamicChanges) :
    model_(std::move(model)),
    solver_(solver),
    options_(solver_->getOptions()),
    solverModel_(solver_->getModel()),
    agent_(std::make_unique<Agent>(solver_)),
    hasDynamicChanges_(hasDynamicChanges),
    changeSequence_(),
    stepCount_(0),
    maxStepCount_(100),
    currentDiscount_(1.0),
    totalDiscountedReward_(0.0),
    actualHistory_(std::make_unique<HistorySequence>()),
    visitedBeliefs_(std::make_unique<std::vector<std::vector<State const*> >>()),
    totalChangingTime_(0.0),
    totalReplenishingTime_(0.0),
    totalImprovementTime_(0.0),
    totalPruningTime_(0.0),
    totalNumHistories_(0)
{
    std::unique_ptr<State> initialState = model_->sampleAnInitState();
    StateInfo* initInfo = solver_->getStatePool()->createOrGetInfo(std::move(initialState));
    HistoryEntry* newEntry = actualHistory_->addEntry();
    newEntry->stateInfo_ = initInfo;
}
Model* Simulator::getModel() const
{
    return model_.get();
}
Agent* Simulator::getAgent() const
{
    return agent_.get();
}
Solver* Simulator::getSolver() const
{
    return solver_;
}

Model* Simulator::getSolverModel() const
{
    return solverModel_;
}


State const* Simulator::getCurrentState() const
{
    return actualHistory_->getLastEntry()->getState();
}
HistorySequence* Simulator::getHistory() const
{
    return actualHistory_.get();
}

std::vector<std::vector<State const*> >* Simulator::getVisitedBeliefs() const
{
    return visitedBeliefs_.get();
}

long Simulator::getStepCount() const
{
    return stepCount_;
}
FloatType Simulator::getTotalChangingTime() const
{
    return totalChangingTime_;
}
FloatType Simulator::getTotalReplenishingTime() const
{
    return totalReplenishingTime_;
}
FloatType Simulator::getTotalImprovementTime() const
{
    return totalImprovementTime_;
}
FloatType Simulator::getTotalPruningTime() const
{
    return totalPruningTime_;
}

long Simulator::getTotalNumHistories() const
{
    return totalNumHistories_;
}


void Simulator::setChangeSequence(ChangeSequence sequence)
{
    changeSequence_ = std::move(sequence);
}
void Simulator::loadChangeSequence(std::string path)
{
    std::ifstream ifs(path);
    setChangeSequence(solver_->getSerializer()->loadChangeSequence(ifs));
    ifs.close();
}
void Simulator::setMaxStepCount(long maxStepCount)
{
    maxStepCount_ = maxStepCount;
}
FloatType Simulator::runSimulation(std::ofstream& os)
{
    bool canDoSimulation = true;
    bool terminalStateReached = false;
    std::pair<FloatType, std::pair<long, FloatType> > simulationResult;
    while (stepSimulation(os)) {

    }
    if (options_->hasVerboseOutput) {
        cout << endl << endl << "Final State:" << endl;
        State const& currentState = *getCurrentState();
        cout << currentState << endl;
        BeliefNode* currentBelief = agent_->getCurrentBelief();
        cout << "Belief #" << currentBelief->getId() << endl;
        model_->drawSimulationState(currentBelief, currentState, cout);
    }
    return totalDiscountedReward_;
}
bool Simulator::stepSimulation(std::ofstream& os)
{   
    if (stepCount_ >= maxStepCount_) {
        return false;
    } else if (model_->isTerminal(*getCurrentState())) {
        return false;
    }    

    std::stringstream prevStream;
    HistoryEntry* currentEntry = actualHistory_->getLastEntry();
    State const* currentState = getCurrentState();
    BeliefNode* currentBelief = agent_->getCurrentBelief();
    if (options_->hasVerboseOutput) {
        cout << endl << endl << "t-" << stepCount_ << endl;              
        /**cout << "Heuristic Value: " << model_->getHeuristicFunction()(currentEntry,
                currentState, currentBelief->getHistoricalData()) << endl;*/
        
        cout << "Belief #" << currentBelief->getId() << endl;

        abt::HistoricalData* data = currentBelief->getHistoricalData();
        if (data != nullptr) {
            cout << endl;
            cout << *data;
            cout << endl;
        }

        model_->drawSimulationState(currentBelief, *currentState, cout);
        prevStream << "Before:" << endl;
        solver_->printBelief(currentBelief, prevStream);
    }
    
    ChangeSequence::iterator iter = changeSequence_.find(stepCount_);
    if (iter != changeSequence_.end()) {
        if (options_->hasVerboseOutput) {
            cout << "Model changing." << endl;
        }
        FloatType changingTimeStart = oppt::clock_ms();
        // Apply all the changes!
        bool noError = handleChanges(iter->second, 
                                     hasDynamicChanges_, 
                                     static_cast<oppt::ABTExtendedOptions const*>(options_)->resetOnChanges);
        // Update the BeliefNode * in case there was a tree reset.
        currentBelief = agent_->getCurrentBelief();
        if (!noError) {
            return false;
        }
        FloatType changingTime = oppt::clock_ms() - changingTimeStart;
        if (options_->hasVerboseOutput) {
            cout << "Changes complete" << endl;
            cout << "Total of " << changingTime << " ms used for changes." << endl;
        }
    }

    FloatType impSolTimeStart = oppt::clock_ms();
    long numHistories = 0;
    if (currentBelief == solver_->getPolicy()->getRoot()) {
        numHistories = solver_->improvePolicy();
    } else {
        numHistories = solver_->improvePolicy(currentBelief);
    }
    totalImprovementTime_ += (oppt::clock_ms() - impSolTimeStart);
    totalNumHistories_ += numHistories;

    //if (numHistories == 0) {
    //    debug::show_message("ERROR: Could not sample any histories!");
    //    return false;
    //}

    if (options_->hasVerboseOutput) {
        std::stringstream newStream;
        newStream << "After:" << endl;
        solver_->printBelief(currentBelief, newStream);
        while (prevStream.good() || newStream.good()) {
            std::string s1, s2;
            std::getline(prevStream, s1);
            std::getline(newStream, s2);
            cout << s1 << std::setw(40 - s1.size()) << "";
            cout << s2 << std::setw(40 - s2.size()) << "";
            cout << endl;
        }

        cout << "Num samples histories: " << numHistories << endl;
    }

    std::unique_ptr<Action> action = agent_->getPreferredAction();
    if (action == nullptr) {
        debug::show_message("ERROR: Could not choose an action!");
        os << "Error: Belief has zero particles" << endl;
        return false;
    }


    Model::StepResult result = model_->generateStep(*currentState, *action);
    /**shared::RobotTransitionParameters* tp = static_cast<shared::RobotTransitionParameters*>(result.transitionParameters.get());
    if (tp->getPropagationResult()->collided &&
            !(static_cast<robot::RobotOptions const*>(options_)->allowCollisions)) {
        cout << "Collision detected" << endl;
        // terminal, (numHistories, success)
        return std::make_pair(false, std::make_pair(numHistories, result.isTerminal));
    }*/
    if (options_->hasVerboseOutput) {
        if (result.isTerminal) {
            cout << "Reached a terminal state!" << endl;
        }
        cout << "Action: " << *result.action << endl;
        cout << "Transition: ";
        if (result.transitionParameters == nullptr) {
            cout << "NULL" << endl;
        } else {
            cout << *result.transitionParameters << endl;
        }

        cout << "Resulting state: " << *result.nextState << endl;

        cout << "Reward: " << result.reward << endl;
        cout << "Observation: " << *result.observation << endl;
        cout << "Discount: " << currentDiscount_ << "; Total Reward: ";
        cout << totalDiscountedReward_ << endl;
    }

    // Replenish the particles.
    FloatType replenishTimeStart = oppt::clock_ms();
    abt::BeliefNode* repNode = nullptr;
    if (!repNode) {
        cout << "Error. Belief has zero particles. Can't continue run." << endl;
        os << "Error: Belief has zero particles" << endl;
        return false;
    }

    FloatType repTime = oppt::clock_ms() - replenishTimeStart;    
    totalReplenishingTime_ += repTime;
    // Update the agent's belief.
    agent_->updateBelief(*result.action, *result.observation);
    //agent_->setCurrentBelief(repNode);
    currentBelief = agent_->getCurrentBelief();
    
    //visitedBeliefs_->push_back(currentParticles);

    //model_->updateModel(result, currentParticles, particleColors);

    // If we're pruning on every step, we do it now.
    if (static_cast<oppt::ABTExtendedOptions const*>(options_)->pruneEveryStep) {
        FloatType pruningTimeStart = oppt::clock_ms();
        long nSequencesDeleted = solver_->pruneSiblings(currentBelief);
        long pruningTime = oppt::clock_ms() - pruningTimeStart;
        totalPruningTime_ += pruningTime;
        if (options_->hasVerboseOutput) {
            cout << "Pruned " << nSequencesDeleted << " sequences in ";
            cout << pruningTime << "ms." << endl;
        }
    }

    currentEntry->action_ = std::move(result.action);
    currentEntry->observation_ = std::move(result.observation);
    currentEntry->immediateReward_ = result.reward;
    currentEntry->transitionParameters_ = std::move(result.transitionParameters);
    totalDiscountedReward_ += currentDiscount_ * result.reward;

    if (currentBelief->getNumberOfParticles() == 0) {
        debug::show_message("ERROR: Resulting belief has zero particles!!");
        os << "Error: Belief has zero particles" << endl;
        return false;
    }

    // Serialize step
    os << "t = " << stepCount_ << endl;
    currentEntry->getState()->serialize(os, "S");
    os << " \n";
    currentEntry->getAction()->serialize(os, "A");
    os << " \n";
    currentEntry->getObservation()->serialize(os, "O");
    os << " \n";
    os << "IMMEDIATE_REWARD: " << currentEntry->getImmediateReward() << endl;
    os << "DISCOUNTED_REWARD: " << currentDiscount_ * currentEntry->getImmediateReward() << endl;

    if (currentEntry->getTransitionParameters() != nullptr) {
        currentEntry->getTransitionParameters()->serialize(os);
    }
    
    os << "NUM HISTORIES: " << numHistories << endl;
    
    stepCount_++;
    StateInfo* nextInfo = solver_->getStatePool()->createOrGetInfo(std::move(result.nextState));
    result.nextState = nullptr;
    currentEntry = actualHistory_->addEntry();
    currentEntry->stateInfo_ = nextInfo;
    currentDiscount_ *= static_cast<oppt::ABTExtendedOptions const*>(options_)->discountFactor;
    return !result.isTerminal;
}

bool Simulator::handleChanges(std::vector<std::unique_ptr<ModelChange>> const& changes,
                              bool areDynamic, bool resetTree)
{
    if (!resetTree) {
        // Set the change root appropriately.
        if (areDynamic) {
            solver_->setChangeRoot(agent_->getCurrentBelief());
        } else {
            solver_->setChangeRoot(nullptr);
        }
    }

    model_->applyChanges(changes, nullptr);

    FloatType startTime = oppt::clock_ms();
    if (resetTree) {
        solverModel_->applyChanges(changes, nullptr);
    } else {
        // The model only needs to inform the solver of changes if we intend to keep the policy.
        solverModel_->applyChanges(changes, solver_);
    }
    totalChangingTime_ += oppt::clock_ms() - startTime;

    // If the current state is deleted, the simulation is broken!
    StateInfo const* lastInfo = actualHistory_->getLastEntry()->getStateInfo();
    if (!solverModel_->isValid(*lastInfo->getState())) {
        debug::show_message("ERROR: Current simulation state has been invalidated!");
        return false;
    }

    // If the changes are not dynamic and a past state is deleted, the simulation is broken.
    if (!areDynamic) {
        for (HistoryEntry::IdType i = 0; i < actualHistory_->getLength() - 1; i++) {
            StateInfo const* info = actualHistory_->getEntry(i)->getStateInfo();
            State const& state = *info->getState();
            if (!solverModel_->isValid(state)) {
                std::ostringstream message;
                message << "ERROR: Impossible simulation history! Includes " << state;
                debug::show_message(message.str());
                return false;
            }
        }
    }

    // Apply the changes, or simply reset the tree.
    startTime = oppt::clock_ms();
    if (resetTree) {
        solver_->resetTree(agent_->getCurrentBelief());
        agent_->setCurrentBelief(solver_->getPolicy()->getRoot());
    } else {
        solver_->applyChanges();
    }
    totalChangingTime_ += oppt::clock_ms() - startTime;
    return true;
}


} /* namespace abt */
