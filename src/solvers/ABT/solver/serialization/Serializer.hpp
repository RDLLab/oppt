/** @file Serializer.hpp
 *
 * Defines Serializer, an abstract base class for serialization of an ABT solver instance.
 */
#ifndef SOLVER_SERIALIZER_HPP_
#define SOLVER_SERIALIZER_HPP_

#include <istream>                      // for istream, ostream
#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

#include "solvers/ABT/solver/Solver.hpp"                   // for Solver

#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "solvers/ABT/solver/abstract-problem/ModelChange.hpp"              // for Observation
#include "solvers/ABT/solver/abstract-problem/State.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionPool.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationPool.hpp"

namespace abt {
class ActionMapping;
class ActionNode;
class BeliefNode;
class BeliefTree;
class HistoricalData;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationMapping;
class StateInfo;
class StatePool;

/** An abstract base class to perform the function of serializing an ABT solver instance.
 *
 * This is useful for loading a pre-generated policy as a starting point for further searching.
 */
class Serializer {
public:
    /** Constructs a serializer without an associated solver. */
    Serializer() :
            solver_(nullptr) {
    }

    /** Constructs a serializer for the given solver. */
    Serializer(Solver *solver) :
            solver_(solver) {
    }

    /** Sets the associated solver of this Serializer to be the given solver. */
    void setSolver(Solver *solver) {
        solver_ = solver;
    }

    /** Returns the associated solver for this Serializer. */
    Solver *getSolver() const {
        return solver_;
    }

    /** Returns the model of the associated solver for this Serializer. */
    Model *getModel() const {
        return solver_->getModel();
    }

    /** Default destructor. */
    virtual ~Serializer() = default;

    /* Copying and moving is disallowed. */
    _NO_COPY_OR_MOVE(Serializer);

    /* --------------- Saving the entirnode.e solver. ----------------- */

    /** Saves (serializes) the full state of the associated ABT solver to the given
     * output stream.
     */
    virtual void save(std::ostream &os) {
        if (!solver_)
            cout << "NO SOLVER" << endl;
        if (!solver_->statePool_)
            cout << "NO STATE POOl" << endl;
        save(*(solver_->statePool_), os);
        save(*(solver_->histories_), os);
        saveActionPool(*(solver_->actionPool_), os);
        saveObservationPool(*(solver_->observationPool_), os);
        save(*(solver_->policy_), os);
    }

    /** Loads (deserializes) the full state of the associated ABT solver from the given
     * input stream.
     */
    virtual void load(std::istream &is) {
        // First we have to initialize all of the required data structures.
    	solver_->initialize();

        load(*(solver_->statePool_), is);
        load(*(solver_->histories_), is);
        solver_->actionPool_ = loadActionPool(is);
        solver_->observationPool_ = loadObservationPool(is);
        load(*(solver_->policy_), is);
    }

    /* ------------------ Saving change sequences -------------------- */
    /** Saves a sequence of model changes. */
    virtual void saveChangeSequence(ChangeSequence const &sequence, std::ostream &os) = 0;
    /** Loads a sequence of model changes. */
    virtual ChangeSequence loadChangeSequence(std::istream &is) = 0;

    /** Saves a single ModelChange. */
    virtual void saveModelChange(ModelChange const &change, std::ostream &os) = 0;
    /** Loads a single ModelChange. */
    virtual std::unique_ptr<ModelChange> loadModelChange(std::istream &is) = 0;

    /* ----------- Saving states/actions/transition parameters/observations ------------- */
    // NOTE: null values need to be handled for saving of states, observations, actions, and
    // transition parameters.

    /** Saves a State. */
    virtual void saveState(State const *state, std::ostream &os) = 0;
    /** Loads a State. */
    virtual std::unique_ptr<State> loadState(std::istream &is) = 0;

    /** Saves an Action. */
    virtual void saveAction(Action const *action, std::ostream &os) = 0;
    /** Loads an Action. */
    virtual std::unique_ptr<Action> loadAction(std::istream &is) = 0;

    /** Saves TransitionParameters. */
    virtual void saveTransitionParameters(TransitionParameters const *tp,
            std::ostream &os) = 0;
    /** Loads TransitionParameters. */
    virtual std::unique_ptr<TransitionParameters> loadTransitionParameters(
            std::istream &is) = 0;

    /** Saves an Observation. */
    virtual void saveObservation(Observation const *obs, std::ostream &os) = 0;
    /** Loads an Observation. */
    virtual std::unique_ptr<Observation> loadObservation(std::istream &is) = 0;


    /* -------------- Saving custom mapping data structures ---------------- */
    /** Saves the pool handling all the actions. */
    virtual void saveActionPool(ActionPool const &actionPool,
            std::ostream &os) = 0;
    /** Loads the pool handling all the actions. */
    virtual  std::unique_ptr<ActionPool> loadActionPool(std::istream &is) = 0;
    /** Saves a mapping of actions to action nodes. */
    virtual void saveActionMapping(ActionMapping const &map,
            std::ostream &os) = 0;
    /** Loads a mapping of actions to action nodes for the given belief. */
    virtual std::unique_ptr<ActionMapping> loadActionMapping(BeliefNode *node,
            std::istream &is) = 0;

    /** Saves the pool handling all the observations. */
    virtual void saveObservationPool(
            ObservationPool const &observationPool, std::ostream &os) = 0;
    /** Loads the pool handling all the observations. */
    virtual std::unique_ptr<ObservationPool> loadObservationPool(
            std::istream &is) = 0;
    /** Saves a mapping of observations to belief nodes. */
    virtual void saveObservationMapping(ObservationMapping const &map,
            std::ostream &os) = 0;
    /** Loads a mapping of observations to belief nodes. */
    virtual std::unique_ptr<ObservationMapping> loadObservationMapping(ActionNode *node,
            std::istream &is) = 0;

    /* --------------- Saving extra belief data ----------------- */
    /** Saves history-derived information about a belief. */
    virtual void saveHistoricalData(HistoricalData const *data, std::ostream &os) = 0;
    /** Loads history-derived information about a belief. */
    virtual std::unique_ptr<HistoricalData> loadHistoricalData(std::istream &is) = 0;

    /* --------------- Saving the state pool ----------------- */
    /** Saves a StateInfo. */
    virtual void save(StateInfo const &wrapper, std::ostream &os) = 0;
    /** Loads a StateInfo. */
    virtual void load(StateInfo &wrapper, std::istream &is) = 0;
    /** Saves a StatePool. */
    virtual void save(StatePool const &pool, std::ostream &os) = 0;
    /** Loads a StatePool. */
    virtual void load(StatePool &pool, std::istream &is) = 0;


    /* --------------- Saving the history sequences ----------------- */
    /** Saves a HistoryEntry. */
    virtual void save(HistoryEntry const &entry, std::ostream &os) = 0;
    /** Loads a HistoryEntry. */
    virtual void load(HistoryEntry &entry, std::istream &is) = 0;
    /** Saves a HistorySequence. */
    virtual void save(HistorySequence const &seq, std::ostream &os) = 0;
    /** Loads a HistorySequence. */
    virtual void load(HistorySequence &seq, std::istream &is) = 0;
    /** Saves a Histories. */
    virtual void save(Histories const &histories, std::ostream &os) = 0;
    /** Loads a Histories. */
    virtual void load(Histories &histories, std::istream &is) = 0;

    /* --------------- Saving the policy tree ----------------- */
    /** Saves an ActionNode. */
    virtual void save(ActionNode const &node, std::ostream &os) = 0;
    /** Loads an ActionNode. */
    virtual void load(ActionNode &node, std::istream &is) = 0;
    /** Saves a BeliefNode. */
    virtual void save(BeliefNode const &node, std::ostream &os) = 0;
    /** Loads a BeliefNode. */
    virtual void load(BeliefNode &node, std::istream &is) = 0;
    /** Saves a BeliefTree. */
    virtual void save(BeliefTree const &tree, std::ostream &os) = 0;
    /** Loads a BeliefTree. */
    virtual void load(BeliefTree &tree, std::istream &is) = 0;

  private:
    /** The ABT solver instance associated with this serializer. */
    Solver *solver_;
};
} /* namespace abt */

#endif /* SOLVER_SERIALIZER_HPP_ */
