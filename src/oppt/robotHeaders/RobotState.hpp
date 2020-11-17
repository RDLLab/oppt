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
#ifndef ___ROBOT_STATE_SPEC_HPP___
#define ___ROBOT_STATE_SPEC_HPP___
#include <iosfwd>                       // for istream, ostream
#include <fstream>
#include <vector>
#include <memory>
#include "oppt/opptCore/core.hpp"

namespace oppt
{

/**
 * A class that represents a weighted RobotState 
 */
class WeightedRobotState: public RobotState
{
public:
    using RobotState::RobotState;
    _NO_COPY_BUT_MOVE(WeightedRobotState)
    /**
     * @brief Default constructor
     */
    WeightedRobotState();
    
    virtual ~WeightedRobotState() {}

    /**
     * @brief Get the weight of this state
     */
    FloatType getWeight() const;

    /**
     * @brief Set the weight of this state
     */
    void setWeight(const FloatType& weight);

protected:
    mutable FloatType weight_;
};


/**
 * A class representing a vector-valued RobotState.
 */
class VectorState: public WeightedRobotState
{
public:
    using WeightedRobotState::WeightedRobotState;
    _NO_COPY_BUT_MOVE(VectorState)
    /**
     * @brief Construct from a VectorFloat
     */    
    VectorState(VectorFloat& stateVector);    

    /**
     * @brief Construct from a const VectorFloat
     */ 
    VectorState(const VectorFloat& stateVector);    
    
    virtual ~VectorState() = default;

    virtual void serialize(std::ostream& os, const std::string prefix = "") const override;

    virtual void print(std::ostream& os) const override;

    virtual FloatType distanceTo(const RobotState& otherState) const override;

    virtual bool equals(const RobotState& otherState) const override;

    virtual std::size_t hash() const override;

    /**
     * @brief Return the raw vector values
     * @return A VectorFloat containing the raw vector values
     */
    VectorFloat asVector() const;    

protected:
    VectorFloat state_;

    int roundingPrecision_ = 6;

    FloatType round_(const FloatType& value) const;

};
}

#endif
