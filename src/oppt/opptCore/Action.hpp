/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef _OPPT_ACTION_HPP_
#define _OPPT_ACTION_HPP_
#include "includes.hpp"

namespace oppt
{
/**
 * An abstract class that represents a POMDP action
 */
class Action
{
public:
    _NO_COPY_BUT_MOVE(Action)
    /**
     * @brief Default constructor
     */
    Action() = default;

    virtual ~Action() = default;

    /**
     * @brief Static cast to type T
     * @tparam T The action type to cast to
     */
    _STATIC_CAST

    /**
     * @brief Serialize this action to the given output stream
     */
    virtual void serialize(std::ostream& os, const std::string& prefix = "") const = 0;

    /**
     * @brief Print the action to the output stream
     */
    virtual void print(std::ostream& os) const = 0;

    friend std::ostream& operator<< (std::ostream& out, const Action& action) {
        action.print(out);
        return out;
    }

    /**
     * @brief Copy the action
     * @return A unique pointer to the copied action
     */
    virtual oppt::ActionUniquePtr copy() const = 0;

    /**
     * @brief Check if another action is equal to this action
     * @param otherAction The other action
     */
    virtual bool equals(const Action& otherAction) const = 0;

    /**
     * @brief Calculate a hash value for this action
     */
    virtual std::size_t hash() const = 0;

    /**
     * @brief Calculate the distance (according to some distance metric) to of another action to this action
     * @param otherAction The other action
     * @return The distance
     */
    virtual FloatType distanceTo(const Action& otherAction) const = 0;

    /**
     * @brief Calculate the distance (according to some distance metric) to of another action to this action
     * @param otherAction The other action
     * @return The distance
     */
    virtual FloatType distanceTo(const oppt::ActionSharedPtr& action) const = 0;

};
}

#endif
