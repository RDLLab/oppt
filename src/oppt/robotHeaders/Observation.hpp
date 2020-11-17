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
#ifndef __OPPT_ROBOT_OBSERVATION_HPP__
#define __OPPT_ROBOT_OBSERVATION_HPP__
#include "oppt/opptCore/core.hpp"

namespace oppt
{
    
/**
 * Specialization of the Observation interface for real vector valued continuous actions
 */
class VectorObservation: public Observation
{
public:
    using Observation::Observation;
    _NO_COPY_BUT_MOVE(VectorObservation)
    /**
     * @brief Construct from a VectorFloat
     */
    VectorObservation(VectorFloat& observationVec);

    /**
     * @brief Construct from a const VectorFloat
     */
    VectorObservation(const VectorFloat& observationVec);

    virtual ~VectorObservation() = default;

    virtual oppt::ObservationUniquePtr copy() const override {
        oppt::ObservationUniquePtr copiedObservation(new VectorObservation(observationVec_));
        return copiedObservation;
    }

    virtual void serialize(std::ostream& os, const std::string& prefix = "") const override;


    virtual void print(std::ostream& os) const override;

    virtual bool equals(const Observation& otherObservation) const override;

    virtual std::size_t hash() const override;

    virtual FloatType distanceTo(const Observation& otherObservation) const override;

    /**
     * @brief Get the underlying action vector
     * @return A VectorFloat with the underlying raw action values
     */
    virtual VectorFloat asVector() const;

protected:
    VectorFloat observationVec_;
    
    long binNumber_;
};

/**
 * Specialization of a VectorObservation for discrete real vector observations. The discretization is
 * represented by a bin number. Two observations with the same bin number are considered equal
 */
class DiscreteVectorObservation: public VectorObservation
{
public:
    using VectorObservation::VectorObservation;
    _NO_COPY_BUT_MOVE(DiscreteVectorObservation)
    /**
     * @brief Construct from a VectorFloat
     */
    DiscreteVectorObservation(VectorFloat& observationValues);

    /**
     * @brief Construct from a const VectorFloat
     */
    DiscreteVectorObservation(const VectorFloat& observationValues);
    
    virtual bool equals(const Observation& otherObservation) const override;
    /**
     * @brief Get the discrete bin number for this observation
     */
    long getBinNumber() const;

    /**
     * @brief Set the discrete bin number for this observation
     */
    void setBinNumber(long binNumber);
};

}

#endif
