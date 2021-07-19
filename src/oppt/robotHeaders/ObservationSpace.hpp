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
#ifndef __OBSERVATION_SPACE_HPP_
#define __OBSERVATION_SPACE_HPP_
#include <vector>
#include <string>
#include "utils.hpp"
#include "oppt/opptCore/core.hpp"
#include "Observation.hpp"
#include "LimitsContainer.hpp"
#include "oppt/opptCore/SpaceComponents.hpp"

namespace oppt
{

struct ObservationSpaceInfo {
public:
    ObservationSpaceInfo() {}

    bool normalized = false;

    // Contains additional information (e.g. 'linear' or 'nonlinear')
    std::string observationModelInfo;

    size_t numDimensions = 1;

    SpaceVariablesPtr spaceVariables;
};

struct standardObservationNormalize;
struct nullObservationNormalize;
class ObservationLimits
{
    friend standardObservationNormalize;
    friend nullObservationNormalize;
public:
    ObservationLimits(const LimitsContainerSharedPtr& limits):
        container_(limits) {}

    virtual ~ObservationLimits() = default;    

    _NO_COPY_BUT_MOVE(ObservationLimits)

    _STATIC_CAST

    virtual bool enforceLimits(ObservationSharedPtr& observation) const = 0;

    virtual const LimitsContainerSharedPtr& getLimits() const {
        return container_;
    }
    
    virtual const LimitsContainerSharedPtr& getRawLimits() const {
        return container_;
    }

protected:
    //Contains the unnormalized limits
    LimitsContainerSharedPtr container_;
};

class VectorObservationLimits: public ObservationLimits
{

public:
    VectorObservationLimits(const LimitsContainerSharedPtr& limits):
        ObservationLimits(limits),
        lowerLimits_(),
        upperLimits_() {
        container_->as<VectorLimitsContainer>()->get(lowerLimits_, upperLimits_);
    }

    virtual ~VectorObservationLimits() {
        lowerLimits_.clear();
        upperLimits_.clear();
        container_.reset();
    }

    virtual bool enforceLimits(ObservationSharedPtr& observation) const override {
        VectorFloat observationVec =
            static_cast<VectorObservation*>(observation.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < observationVec.size(); i++) {
            if (observationVec[i] < lowerLimits_[i]) {
                enforced = true;
                observationVec[i] = lowerLimits_[i];
            } else if (observationVec[i] > upperLimits_[i]) {
                enforced = true;
                observationVec[i] = upperLimits_[i];
            }
        }

        observation = std::make_shared<VectorObservation>(observationVec);
        return enforced;
    }

protected:
    VectorFloat lowerLimits_;
    VectorFloat upperLimits_;
};

class NormalizedVectorObservationLimits: public VectorObservationLimits
{
public:
    NormalizedVectorObservationLimits(const LimitsContainerSharedPtr& limits):
        VectorObservationLimits(limits) {
        VectorFloat lowerLimits(lowerLimits_.size(), 0);
        VectorFloat upperLimits(upperLimits_.size(), 1);
        normalizedLimitsContainer_ =
            std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);

    }

    virtual ~NormalizedVectorObservationLimits() {
        normalizedLimitsContainer_.reset();
    }

    virtual bool enforceLimits(ObservationSharedPtr& observation) const override {
        VectorFloat observationVec =
            static_cast<VectorObservation*>(observation.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < observationVec.size(); i++) {
            if (observationVec[i] < 0) {
                enforced = true;
                observationVec[i] = 0;
            } else if (observationVec[i] > 1) {
                enforced = true;
                observationVec[i] = 1;
            }
        }

        observation = std::make_shared<VectorObservation>(observationVec);
        return enforced;
    }

    virtual const LimitsContainerSharedPtr& getLimits() const override {
        return normalizedLimitsContainer_;
    }

private:
    LimitsContainerSharedPtr normalizedLimitsContainer_;

};

class ObservationNormalizer
{
public:  
    virtual ~ObservationNormalizer() = default;

    virtual ObservationSharedPtr normalizeObservation(const ObservationSharedPtr& observation) const = 0;

    virtual ObservationSharedPtr denormalizeObservation(const ObservationSharedPtr& observation) const = 0;

    void setObservationLimits(ObservationLimitsSharedPtr& observationLimits) {
        observationLimits_ = observationLimits;
    }

protected:
    ObservationLimitsSharedPtr observationLimits_ = nullptr;
};

/**
 * Represents the POMDP observation space
 */
class ObservationSpace
{
public:
    /**
     * @brief Construct from ObservationSpaceInfo
     * @param observationSpaceInfo The ObservationSpaceInfo this observation space is constructed from
     */
    ObservationSpace(const ObservationSpaceInfo& observationSpaceInfo);

    virtual ~ObservationSpace() = default;

    template<class T>
    T* as() {
        return dynamic_cast<T*>(this);
    }

    /**
     * @brief Get the observation space type
     */
    virtual SpaceType::StateSpaceType getType() const = 0;

    /**
     * @brief Gets the bounds of the observation space
     */
    ObservationLimitsSharedPtr getObservationLimits() const;

    /**
     * @brief Forces an observation into the observation space bounds
     */
    bool enforceObservationLimits(ObservationSharedPtr& observation) const;

    /**
     * @brief Get the number of dimensions
     */
    size_t getNumDimensions() const;

    /**
     * @brief Get the ObservationSpaceInfo this observation space was constructed from
     */
    const ObservationSpaceInfo getInfo() const;

    virtual VectorFloat getOrigin() const = 0;

    /**
     * @brief Normalizes an observation to [0, 1]
     * If the oppt::ProblemEnvironmentOptions::normalizedSpaces is false, this method has no effect     
     * @param observation The observation to normalize
     * @return A shared pointer to the normalized observation
     */    
    ObservationSharedPtr normalizeObservation(const ObservationSharedPtr& observation) const {
        if (!observationNormalizer_)
            return observation;
        return observationNormalizer_->normalizeObservation(observation);

    }

    /**
     * @brief Denormalizes an observation
     * If the oppt::ProblemEnvironmentOptions::normalizedSpaces is false, this method has no effect     
     * @param observation The observation to denormalize
     * @return A shared pointer to the denormalized observation
     */    
    ObservationSharedPtr denormalizeObservation(const ObservationSharedPtr& observation) const {
        if (!observationNormalizer_)
            return observation;
        return observationNormalizer_->denormalizeObservation(observation);
    }

    /**
     * @brief Set a custom oppt::ObservationNormalizer
     */
    virtual void setObservationNormalizer(std::unique_ptr<ObservationNormalizer> observationNormalizer);

protected:
    size_t dimension_;

    ObservationSpaceInfo observationSpaceInfo_;

    ObservationLimitsSharedPtr observationLimits_;
    
    ObservationLimitsSharedPtr denormalizedObservationLimits_;

    std::unique_ptr<ObservationNormalizer> observationNormalizer_ = nullptr;
};

/**
 * Specialization of a oppt::ObservationNormalizer for vector observation spaces
 */
class VectorObservationNormalizer: public ObservationNormalizer {
public:
    virtual ~VectorObservationNormalizer() = default;

    virtual ObservationSharedPtr normalizeObservation(const ObservationSharedPtr& observation) const override;

    virtual ObservationSharedPtr denormalizeObservation(const ObservationSharedPtr& observation) const override;

};

/**
 * A specialization of the abstract ObservationSpace class for vector valued
 * observation spaces
 */
class VectorObservationSpace: public ObservationSpace
{
public:
    VectorObservationSpace(const ObservationSpaceInfo& observationSpaceInfo);

    virtual ~VectorObservationSpace() {}

    virtual SpaceType::StateSpaceType getType() const override;

    /**
     * @brief Constructs the observation bounds
     * @param lowerLimits Vector containing the lower limits of the observation space
     * @param upperLimits Vector containing the upper limits of the observation space
     */
    virtual void makeObservationLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits);

    virtual VectorFloat getOrigin() const override;
};

}

#endif
