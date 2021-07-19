/** @file ChangeFlags.hpp
 *
 * Contains an enum class used to define flags for the different types of changes that can
 * apply at a single state and, in turn, a history entry with that state.
 *
 * This file also defines some basic operators (Bitwise OR, AND, NOT) to allow combinations of
 * flags, even thoug this class is used as a strict enum.
 */
#ifndef SOLVER_CHANGEFLAGS_HPP_
#define SOLVER_CHANGEFLAGS_HPP_

#include <cstdint>

#include <initializer_list>

namespace abt {
/** The integer type used to store changes. */
typedef uint8_t ChangeFlagsIntType;

/** An enumeration that defines the types of flags that can apply at a single state, and, in turn,
 * at all of the history entries associated with that state.
 *
 * This is a strict enum (enum class) in order to ensure that the type is used carefully. Instead
 * of allowing them to be treated as a numeric type, we provide operators to allow bitwise or
 * and bitwise and operations.
 */
enum class ChangeFlags : ChangeFlagsIntType {
    // No changes
    UNCHANGED = 0x000,

    /* --------------- Flags to apply within the state pool. --------------- */
    // Change in the reward value.
    REWARD = 0x001,

    // Different transition.
    TRANSITION = 0x002,

    // Different transition from the previous state;
    // TRANSITION_BEFORE(e) implies TRANSITION(prev(e))
    TRANSITION_BEFORE = 0x004,

    // This state is deleted; this also means the prior transition must change.
    // DELETED(e) implies TRANSITION_BEFORE(e)
    DELETED = 0x008,

    // Recalculate observation for this current time step.
    OBSERVATION = 0x010,

    // Recalculate observation for the previous time step (o depends on s').
    // OBSERVATION_BEFORE(e) implies OBSERVATION(prev(e))
    OBSERVATION_BEFORE = 0x020,

    // Recalculate the heuristic value iff this state is at the end of a sequence.
    HEURISTIC = 0x040,
};

/** Bitwise OR assignment operator. */
inline ChangeFlags &operator|=(ChangeFlags &lhs, ChangeFlags const &rhs) {
    lhs = static_cast<ChangeFlags>(
            static_cast<ChangeFlagsIntType>(lhs) | static_cast<ChangeFlagsIntType>(rhs));
    return lhs;
}

/** Bitwise OR operator. */
inline ChangeFlags operator|(ChangeFlags lhs, ChangeFlags const &rhs) {
    lhs |= rhs;
    return lhs;
}

/** Bitwise AND assignment operator. */
inline ChangeFlags &operator&=(ChangeFlags &lhs, ChangeFlags const &rhs) {
    lhs = static_cast<ChangeFlags>(
            static_cast<ChangeFlagsIntType>(lhs) & static_cast<ChangeFlagsIntType>(rhs));
    return lhs;
}

/** Bitwise AND operator. */
inline ChangeFlags operator&(ChangeFlags lhs, ChangeFlags const &rhs) {
    lhs &= rhs;
    return lhs;
}

/** Bitwise NOT operator. */
inline ChangeFlags operator~(ChangeFlags const &cf) {
    return static_cast<ChangeFlags>(~static_cast<ChangeFlagsIntType>(cf));
}

namespace changes {
/** Returns true iff all of the given flags are set in the given value. */
inline bool has_flags(ChangeFlags value, ChangeFlags flags) {
    return (value & flags) == flags;
}
} /* namespace changes */
} /* namespace abt */

#endif /* SOLVER_CHANGEFLAGS_HPP_ */
