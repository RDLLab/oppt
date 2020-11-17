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
#ifndef _OPPT_OBSERVATION_HPP_
#define _OPPT_OBSERVATION_HPP_
#include "includes.hpp"

namespace oppt
{
/**
* An abstract class that represents a POMDP observation
*/
class Observation
{
public:
    _NO_COPY_BUT_MOVE(Observation);
    /**
     * @brief Default constructor
     */
    Observation() = default;

    virtual ~Observation() = default;

    /**
     * @brief Static cast to type T
     * @tparam T The action type to cast to
     */
    _STATIC_CAST

    /**
     * @brief Serialize this observation to the given output stream
     */
    virtual void serialize(std::ostream& os, const std::string& prefix = "") const = 0;

    /**
     * @brief Print the observation to the output stream
     */
    virtual void print(std::ostream& os) const = 0;

    friend std::ostream& operator<< (std::ostream& out, const Observation& observation) {
        observation.print(out);
        return out;
    }

    /**
     * @brief Copy the observation
     * @return A unique pointer to the copied observation
     */
    virtual oppt::ObservationUniquePtr copy() const = 0;

    /**
     * @brief Check if another observation is equal to this observation
     * @param otherObservation The other observation
     */
    virtual bool equals(const Observation& otherObservation) const = 0;

    /**
     * @brief Calculate a hash value for this observation
     */
    virtual std::size_t hash() const = 0;

    /**
     * @brief Calculate the distance (according to some distance metric) to of another observation
     * @param otherObservation The other observation
     * @return The distance
     */
    virtual FloatType distanceTo(const Observation& otherObservation) const = 0;

};
}

#endif
