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
#ifndef __OPPT_PARTICLE_FILTER_HPP__
#define __OPPT_PARTICLE_FILTER_HPP__
#include "oppt/opptCore/core.hpp"
#include "oppt/utils/include/Sampler.hpp"
#include "oppt/robotEnvironment/include/RobotEnvironment.hpp"
#include <initializer_list>

namespace oppt
{

/** Forward declaration of oppt::ParticleSet */
class ParticleSet;

/**
 * A class that represents a single belief particle
 */
class Particle
{
public:
    friend class ParticleSet;
    _NO_COPY_BUT_MOVE(Particle)

    virtual ~Particle() = default;
    
    /** @brief Construct from a oppt::RobotStateSharedPtr */
    Particle(const RobotStateSharedPtr& state);

    /** @brief Construct from a oppt::RobotStateSharedPtr and an associated weight*/
    Particle(const RobotStateSharedPtr& state, const FloatType& weight);

    /** @brief Set the weight of the particle */
    void setWeight(const FloatType& weight);

    /** @brief Get the weight of the particle */
    const FloatType getWeight() const;

    /** @brief Get the oppt::RobotStateSharedPtr the particle represents */
    const RobotStateSharedPtr getState() const;

protected:
    const RobotStateSharedPtr state_;

    FloatType weight_;

};

/** @brief std::shared_ptr to a oppt::Particle */
typedef std::shared_ptr<Particle> ParticlePtr;
/** @brief std::vector of oppt::ParticlePtr */
typedef std::vector<ParticlePtr> VectorParticles;

/**
 * A particle filter request
 */
class FilterRequest
{
public:
    /**
     * @brief Constructor
     */
    FilterRequest() = default;

    _NO_COPY_BUT_MOVE(FilterRequest)

    virtual ~FilterRequest() = default;

    /**
     * @brief The robot environment in which the filtering shoud happen
     */
    RobotEnvironment* robotEnvironment;

    /**
     * @brief A vector of oppt::RobotStateSharedPtr representing the previous particles
     */
    VectorParticles previousParticles;

    /**
     * @brief A vector of oppt::RobotStateSharedPtr representing the set of already
     * calculated next particles. These particles only need re-weighting
     */
    VectorParticles currentNextParticles;

    /**
     * @brief The minimum number of filtered particles
     */
    unsigned int numParticles = 0;

    /**
     * @brief Allow terminal states to be part of the next belief
     */
    bool allowTerminalStates = false;

    /**
     * @brief Allow zero weight particle to be part of the next belief
     */
    bool allowZeroWeightParticles = false;

    /**
     * @brief The Action that was applied
     */
    const Action *action = nullptr;

    /**
     * @brief The Observation that was received
     */
    const Observation *observation = nullptr;

    /**
     * @brief If this parameter is set to false, particles that collide with an body are not considered
     */
    bool allowCollisions = false;

    RandomEnginePtr randomEngine = nullptr;

};

/**
 * A class representing a particle filter result
 */
class FilterResult
{
public:
    FilterResult() = default;

    _NO_COPY_BUT_MOVE(FilterResult)

    virtual ~FilterResult() = default;

    /** @brief The filtered particles */
    VectorParticles particles;
};

class ParticleFilter;

/**
 * A class representing a set of particles.
 * Provides simple sampling methods to sample particles from this set
 */
class ParticleSet
{
public:
    friend class ParticleFilter;
    /**
     * Construct using a pointer to a oppt::FilterRequest
     */
    ParticleSet(const FilterRequest* filterRequest);

    /**
     * Construct an empty particle set
     */
    ParticleSet();

    /**
     * Set the vector of particles managed by this set
     */
    void setParticles(const VectorParticles& particles);

    /**
     * Sample a particle from the particle set according to their weight
     */
    const VectorParticles sampleWeighted(RandomEnginePtr& randomEngine, const unsigned int& numSamples) const;

    /**
     * Uniformly sample a particle from the particle set
     */
    const VectorParticles sampleUniform(RandomEnginePtr& randomEngine, const unsigned int& numSamples) const;

private:
    VectorParticles particles_;

    WeightedDiscreteSampler weightedParticleSampler_;

};

/**
 * This class implements the sequential importance resampling particle filter
 */
class ParticleFilter: public Filter
{
public:
    /**
     * @brief Constructor
     */
    ParticleFilter();

    /**
     * @brief virtual default destructor
     */
    virtual ~ParticleFilter() = default;

    _NO_COPY_BUT_MOVE(ParticleFilter)

    /**
     * @brief Perform particle filtering
     * @param filterRequest A pointer to a FilterRequest containing the current set of particles
     *
     * @return FilterResultPtr a point to a FilterResult,
     * containing the filtered particles and their new weights
     */
    virtual FilterResultPtr filter(const FilterRequestPtr& filterRequest) override;

    virtual FilterResultPtr propagateParticles(const FilterRequestPtr& filterRequest);
};

}

#endif
