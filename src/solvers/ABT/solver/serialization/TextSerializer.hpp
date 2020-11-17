/** @file TextSerializer.hpp
 *
 * Defines the TextSerializer, which provides a basic Serializer implementation that uses
 * a human-readable text-based approach to serializing all of the data structures.
 *
 * Note that this is not a full implementation, as custom mappings, and custom classes for
 * changes / states / actions / transition parameters / observations / historical data will require
 * implementations of the associated serialization methods.
 *
 * Note that if you don't implement one of these (e.g. you leave historical data as nullptr for
 * each belief node) you can simply use the default do-nothing implementation for serialization.
 */
#ifndef SOLVER_TEXTSERIALIZER_HPP_
#define SOLVER_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for ostream, istream
#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

#include "Serializer.hpp"               // for Serializer

#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "solvers/ABT/solver/abstract-problem/State.hpp"

namespace abt {
class ActionMapping;
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationMapping;
class Solver;
class StateInfo;
class StatePool;

/** Implements a text-based serialization of the core solver classes, i.e.
 *
 * StatePool/StateInfo;
 * Histories/HistorySequence/HistoryEntry;
 * BeliefTree/BeliefNode/ActionNode;
 *
 * A default implementation for serialization of change sequences is also provided; it is likely
 * that this will not need to be overridden.
 *
 * This class also provides default do-nothing implementations for serialization of ModelChange,
 * TransitionParameters, and HistoricalData - because these classes are not mandatory and may
 * simply be represented by null pointers, you may simply not use those classes and allow the
 * default serialization.
 */
class TextSerializer : virtual public Serializer {
  public:
    TextSerializer() = default;
    virtual ~TextSerializer() = default;
    _NO_COPY_OR_MOVE(TextSerializer);

    /* ------------------ Saving change sequences -------------------- */
    virtual void saveChangeSequence(ChangeSequence const &sequence, std::ostream &os) override;
    virtual ChangeSequence loadChangeSequence(std::istream &is) override;
    virtual void saveModelChange(ModelChange const &change, std::ostream &os) override;
    virtual std::unique_ptr<ModelChange> loadModelChange(std::istream &is) override;

    /* ------------------ Saving transition parameters -------------------- */
    virtual void saveTransitionParameters(TransitionParameters const *tp,
            std::ostream &os) override;
    virtual std::unique_ptr<TransitionParameters> loadTransitionParameters(
            std::istream &is) override;

    /* ------------------ Saving historical data -------------------- */
    virtual void saveHistoricalData(HistoricalData const *data, std::ostream &os) override;
    virtual std::unique_ptr<HistoricalData> loadHistoricalData(std::istream &is) override;

    /* ------------------ Saving the state pool -------------------- */
    virtual void save(StateInfo const &wrapper, std::ostream &os) override;
    virtual void load(StateInfo &wrapper, std::istream &is) override;
    virtual void save(StatePool const &pool, std::ostream &os) override;
    virtual void load(StatePool &pool, std::istream &is) override;

    /* ------------------ Saving the histories -------------------- */
    virtual void save(HistoryEntry const &entry, std::ostream &os) override;
    virtual void load(HistoryEntry &entry, std::istream &is) override;
    virtual void save(HistorySequence const &seq, std::ostream &os) override;
    virtual void load(HistorySequence &seq, std::istream &is) override;
    virtual void save(Histories const &histories, std::ostream &os) override;
    virtual void load(Histories &histories, std::istream &is) override;

    /* ------------------ Saving the policy tree -------------------- */
    virtual void save(ActionNode const &node, std::ostream &os) override;
    virtual void load(ActionNode &node, std::istream &is) override;
    virtual void save(BeliefNode const &node, std::ostream &os) override;
    virtual void load(BeliefNode &node, std::istream &is) override;
    virtual void save(BeliefTree const &tree, std::ostream &os) override;
    virtual void load(BeliefTree &tree, std::istream &is) override;

    /** The number of particles to display in each line of text. */
    static const int NUM_PARTICLES_PER_LINE = 5;

    // Minimum column width methods - these allow all of the text to line up nicely.
    // The default for each of these methods is simply to return 0.

    /** The minimum column width to use for serializing actions. */
    virtual int getActionColumnWidth();
    /** The minimum column width to use for serializing transition parameters. */
    virtual int getTPColumnWidth();
    /** The minimum column width to use for serializing observations. */
    virtual int getObservationColumnWidth();
};
} /* namespace abt */

#endif /* SOLVER_TEXTSERIALIZER_HPP_ */
