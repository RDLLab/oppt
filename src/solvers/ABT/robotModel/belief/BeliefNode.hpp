#ifndef __ABT_BELIEF_NODE_HPP__
#define __ABT_BELIEF_NODE_HPP__
#include "oppt/opptCore/core.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"
#include "solvers/ABT/robotModel/RobotTransitionParameters.hpp"

namespace abt
{
class OPPTHistoryEntry;
}


namespace oppt
{

class EvaluatedActionsMapEntry
{
public:
    EvaluatedActionsMapEntry();

    void setTransitionParameters(const abt::TransitionParameters* transitionParameters);

    void setState(const abt::State* state);

    void setPropagatedState(const abt::State* state);

    void setObservation(const abt::Observation* observation);

    void setHeuristic(const FloatType& heuristic);

    void setReward(const FloatType& reward);

    void setTerminal(const bool& terminal);

    const abt::State* getState() const;

    const abt::State* getPropagatedState() const;

    const abt::Observation* getObservation() const;

    const abt::TransitionParameters* getTransitionParameters() const;

    const FloatType getHeuristic() const;

    const FloatType getReward() const;

    const bool getTerminal() const;

private:
    std::unique_ptr<abt::TransitionParameters> transitionParameters_;

    std::unique_ptr<abt::State> state_;

    std::unique_ptr<abt::State> propagatedState_;

    std::unique_ptr<abt::Observation> observation_;

    FloatType heuristic_;

    FloatType reward_;

    bool terminal_;

};

typedef std::vector<std::unique_ptr<EvaluatedActionsMapEntry>> EvaluatedActionStateVector;
typedef std::unordered_map<long, EvaluatedActionStateVector> EvaluatedActionStateEntries;


class EvaluatedActionsMap
{
public:
    EvaluatedActionsMap();

    bool contains(long& key) const;

    size_t size() const ;

    void insert(long& key, const abt::State* state, const abt::Model::StepResult& stepResult);

    const EvaluatedActionStateVector* get(long& key) const;

private:
    EvaluatedActionStateEntries evaluatedActionsMap_;

};

class BeliefNode: public abt::BeliefNode
{
public:
    friend class abt::OPPTHistoryEntry;
    /** Constructs a new belief node with no ID, and no parent entry, that will belong to the
     * given solver.
     */
    BeliefNode(abt::Solver* solver);
    /** Constructs a new belief node with no ID (-1), with the given mapping entry as
     * its parent, and which will belong to the given solver.
     */
    BeliefNode(abt::ObservationMappingEntry* parentEntry, abt::Solver* solver);
    /** Constructs a new belief node with the given ID,with the given mapping entry as its parent,
     * and which will belong to the given solver.
     */
    BeliefNode(long id, abt::ObservationMappingEntry* parentEntry, abt::Solver* solver);

    virtual ~BeliefNode();

    virtual void clear();

    unsigned int getTotalNumDynamicsUsed() const;

    // Default destructor; copying and moving disallowed!
    //virtual ~BeliefNode() = default;
    _NO_COPY_OR_MOVE(BeliefNode);

protected:
    virtual void addParticle(abt::HistoryEntry* newHistEntry) override;
    virtual void removeParticle(abt::HistoryEntry* histEntry) override;
    virtual void registerParticle(abt::HistoryEntry* newHistEntry);
    virtual void deregisterParticle(abt::HistoryEntry* histEntry);

private:

    // The total number of propagation from this node
    mutable unsigned int totalNumDynamicsUsed_;
};

}

#endif
