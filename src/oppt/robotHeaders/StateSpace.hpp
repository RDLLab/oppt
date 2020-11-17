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
#ifndef __OPPT_STATE_SPACE_HPP__
#define __OPPT_STATE_SPACE_HPP__
#include "oppt/opptCore/core.hpp"
#include "RobotState.hpp"
#include "LimitsContainer.hpp"
#include "oppt/opptCore/SpaceComponents.hpp"

namespace oppt
{

/**
 * @brief Different types of variables a space can consist of
 */
struct StateSpaceInfo {
    // 'discrete' or 'continuous'
    std::string type = "discrete";

    bool normalized = false;

    size_t numDimensions = 1;

    SpaceVariablesPtr spaceVariables;
};

struct DiscreteStateSpaceInfo: public StateSpaceInfo {
    DiscreteStateSpaceInfo():
        StateSpaceInfo() {}

    size_t numDiscreteStates = 1;
};

class StateLimits
{


public:
    StateLimits(const LimitsContainerSharedPtr& limits):
        container_(limits) { 
    }

    _NO_COPY_BUT_MOVE(StateLimits)

    _STATIC_CAST

    virtual ~StateLimits() {
        container_.reset();
    }

    virtual bool enforceLimits(oppt::RobotStateSharedPtr& state) const = 0;

    virtual bool insideStateLimits(const oppt::RobotStateSharedPtr& state) const = 0;

    virtual const LimitsContainerSharedPtr& getLimits() const {
        return container_;
    }

    virtual const LimitsContainerSharedPtr& getRawLimits() const {
        return container_;
    }

protected:
    LimitsContainerSharedPtr container_;
};

class VectorStateLimits: public StateLimits
{
public:
    VectorStateLimits(const LimitsContainerSharedPtr& limits):
        StateLimits(limits),
        lowerLimits_(),
        upperLimits_() {
        container_->as<VectorLimitsContainer>()->get(lowerLimits_, upperLimits_);
    }

    virtual ~VectorStateLimits() = default;

    virtual bool enforceLimits(oppt::RobotStateSharedPtr& state) const override {
        VectorFloat stateVec = static_cast<oppt::VectorState*>(state.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < stateVec.size(); i++) {
            if (stateVec[i] < lowerLimits_[i]) {
                enforced = true;
                stateVec[i] = lowerLimits_[i];
            } else if (stateVec[i] > upperLimits_[i]) {
                enforced = true;
                stateVec[i] = upperLimits_[i];
            }
        }

        GazeboWorldStatePtr worldState = state->getGazeboWorldState();
        auto subStates = state->getSubStates();
        state = std::make_shared<oppt::VectorState>(stateVec);
        state->setGazeboWorldState(worldState);
        state->setSubStates(subStates);
        return enforced;
    }

    virtual bool insideStateLimits(const oppt::RobotStateSharedPtr& state) const override {
        VectorFloat stateVec = static_cast<oppt::VectorState*>(state.get())->asVector();
        for (size_t i = 0; i < stateVec.size(); i++) {
            if (stateVec[i] < lowerLimits_[i]) {
                return false;
            } else if (stateVec[i] > upperLimits_[i]) {
                return false;
            }
        }

        return true;
    }

protected:
    VectorFloat lowerLimits_;
    VectorFloat upperLimits_;
};

class NormalizedVectorStateLimits: public VectorStateLimits
{
public:
    NormalizedVectorStateLimits(const LimitsContainerSharedPtr& limits):
        VectorStateLimits(limits),
        normalizedLimitsContainer_(nullptr) {
        VectorFloat lowerLimits(lowerLimits_.size(), 0);
        VectorFloat upperLimits(upperLimits_.size(), 1);
        normalizedLimitsContainer_ =
            std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);

    }

    virtual ~NormalizedVectorStateLimits() {
        normalizedLimitsContainer_.reset();
    }

    virtual bool enforceLimits(oppt::RobotStateSharedPtr& state) const override {
        VectorFloat stateVec = static_cast<oppt::VectorState*>(state.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < stateVec.size(); i++) {
            if (stateVec[i] < 0) {
                enforced = true;
                stateVec[i] = 0;
            } else if (stateVec[i] > 1) {
                enforced = true;
                stateVec[i] = 1;
            }
        }

        GazeboWorldStatePtr worldState = state->getGazeboWorldState();
        auto subStates = state->getSubStates();
        state = std::make_shared<oppt::VectorState>(stateVec);
        state->setGazeboWorldState(worldState);
        state->setSubStates(subStates);
        return enforced;
    }

    virtual bool insideStateLimits(const oppt::RobotStateSharedPtr& state) const override {
        VectorFloat stateVec = static_cast<oppt::VectorState*>(state.get())->asVector();
        for (size_t i = 0; i < stateVec.size(); i++) {
            if (stateVec[i] < 0) {
                return false;
            } else if (stateVec[i] > 1) {
                return false;
            }
        }

        return true;
    }

    virtual const LimitsContainerSharedPtr& getLimits() const override {
        return normalizedLimitsContainer_;
    }

private:
    LimitsContainerSharedPtr normalizedLimitsContainer_;
};

class StateNormalizer
{
public:
    virtual ~StateNormalizer() = default;

    virtual RobotStateSharedPtr normalizeState(const RobotStateSharedPtr& state) const = 0;

    virtual RobotStateSharedPtr denormalizeState(const RobotStateSharedPtr& state) const = 0;

    void setStateLimits(StateLimitsSharedPtr& stateLimits) {
        stateLimits_ = stateLimits;
    }

protected:
    StateLimitsSharedPtr stateLimits_ = nullptr;

};

/**
 * Represents a POMDP state space
 */
class StateSpace
{
public:
    /**
     * @brief Construct from StateSpaceInfo
     */
    StateSpace(const StateSpaceInfo& stateSpaceInfo):
        stateSpaceInfo_(stateSpaceInfo),
        stateLimits_(nullptr),
        denormalizedStateLimits_(nullptr),
        dimensions_(stateSpaceInfo.numDimensions),
        distanceMetric_(nullptr) {

    }

    virtual ~StateSpace() {
        stateLimits_.reset();
    }

    _NO_COPY_BUT_MOVE(StateSpace)

    _STATIC_CAST

    /**
     * @brief Gets the type of the state space
     */
    virtual SpaceType::StateSpaceType getType() const = 0;

    virtual VectorFloat getOrigin() const = 0;

    /**
     * @brief Calculates the distance between two states according to the distance metric
     */
    virtual FloatType distance(const oppt::RobotStateSharedPtr state1,
                               const oppt::RobotStateSharedPtr state2) const {
        return distanceMetric_(state1, state2);
    }

    /**
     * @brief Forces a state into the space bounds
     */
    bool enforceStateLimits(oppt::RobotStateSharedPtr& state) const {
        if (!stateLimits_) {
            WARNING("Trying to enforce state limits, but no limits set");
            return false;

        } else {
            return stateLimits_->enforceLimits(state);
        }
    }

    /**
     * @brief Get the StateSpaceInfo this state space was constructed from
     */
    const StateSpaceInfo getInfo() const {
        return stateSpaceInfo_;
    }

    /**
     * @brief Gets the state limits
     */
    StateLimitsSharedPtr getStateLimits() const {
        return stateLimits_;
    }

    /**
     * @brief Checks if a state is inside the state limits
     */
    bool insideStateLimits(const oppt::RobotStateSharedPtr& state) const {
        return stateLimits_->insideStateLimits(state);
    }

    /**
     * @brief Gets the number of dimensions
     */
    unsigned int getNumDimensions() const {
        return dimensions_;
    }

    /**
     * @brief Sets the DistanceMetric to use when calling distance()
     * @param distanceMetric The DistanceMetric
     */
    void setDistanceMetric(const DistanceMetric& distanceMetric) {
        distanceMetric_ = distanceMetric;
    }

    /**
     * @brief Calculates a linear interpolation between two states
     * @param state1 The first RobotState at t = 0
     * @param state2 The second RobotState at t = 1
     * @param t Must be in the interval [0, 1]
     *
     */
    virtual RobotStateSharedPtr interpolate(const RobotStateSharedPtr& state1,
                                            const RobotStateSharedPtr& state2,
                                            const FloatType& t) const = 0;


    /**
     * @brief Samples a RobotState uniformly at random
     */
    virtual RobotStateSharedPtr sampleUniform(std::shared_ptr<std::default_random_engine> randGen) const = 0;

    /**
     * @brief Normalizes a RobotState to [0, 1]
     * If the oppt::ProblemEnvironmentOptions::normalizedSpaces is false, this method has no effect     
     * @param state The RobotState to normalize
     * @return A shared pointer to the normalized RobotState
     */    
    RobotStateSharedPtr normalizeState(const RobotStateSharedPtr& state) const {
        if (!stateNormalizer_)
            return state;
        return stateNormalizer_->normalizeState(state);
    }

    /**
     * @brief Denormalizes a RobotState
     * If the oppt::ProblemEnvironmentOptions::normalizedSpaces is false, this method has no effect     
     * @param state The RobotState to denormalize
     * @return A shared pointer to the denormalized RobotState
     */    
    RobotStateSharedPtr denormalizeState(const RobotStateSharedPtr& state) const {
        if (!stateNormalizer_)
            return state;
        return stateNormalizer_->denormalizeState(state);
    }

    /**
     * @brief Set a custom oppt::StateNormalizer
     */
    virtual void setStateNormalizer(std::unique_ptr<StateNormalizer> stateNormalizer) {
        stateNormalizer_ = std::move(stateNormalizer);
    }

protected:
    StateLimitsSharedPtr stateLimits_;

    StateLimitsSharedPtr denormalizedStateLimits_;

    unsigned int dimensions_;

    DistanceMetric distanceMetric_;

    StateSpaceInfo stateSpaceInfo_;

    std::unique_ptr<StateNormalizer> stateNormalizer_ = nullptr;
};

/**
 * Specialization of a oppt::StateNormalizer for vector state spaces
 */
class VectorStateNormalizer: public StateNormalizer {
public:
    virtual ~VectorStateNormalizer() = default;

    virtual RobotStateSharedPtr normalizeState(const RobotStateSharedPtr& state) const override {
        VectorFloat lowerStateLimits;
        VectorFloat upperStateLimits;
        stateLimits_->getRawLimits()->as<VectorLimitsContainer>()->get(lowerStateLimits, upperStateLimits);
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        for (size_t i = 0; i < stateVec.size(); i++) {
            stateVec[i] = (stateVec[i] - lowerStateLimits[i]) / (upperStateLimits[i] - lowerStateLimits[i]);
        }

        RobotStateSharedPtr normalizedState = std::make_shared<VectorState>(stateVec);
        GazeboWorldStatePtr ws = state->getGazeboWorldState();
        auto subStates = state->getSubStates();
        normalizedState->setGazeboWorldState(ws);
        normalizedState->setSubStates(subStates);
        return normalizedState;
    }

    virtual RobotStateSharedPtr denormalizeState(const RobotStateSharedPtr& state) const override {
        VectorFloat lowerStateLimits;
        VectorFloat upperStateLimits;
        stateLimits_->getRawLimits()->as<VectorLimitsContainer>()->get(lowerStateLimits, upperStateLimits);
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        for (size_t i = 0; i < stateVec.size(); i++) {
            stateVec[i] = stateVec[i] * (upperStateLimits[i] - lowerStateLimits[i]) + lowerStateLimits[i];
        }

        RobotStateSharedPtr denormalizedState = std::make_shared<VectorState>(stateVec);
        GazeboWorldStatePtr ws = state->getGazeboWorldState();
        auto subStates = state->getSubStates();
        denormalizedState->setGazeboWorldState(ws);
        denormalizedState->setSubStates(subStates);
        return denormalizedState;
    }

};

class VectorStateSpace: public StateSpace
{
public:
    VectorStateSpace(const StateSpaceInfo& stateSpaceInfo):
        StateSpace(stateSpaceInfo) {
        if (stateSpaceInfo.normalized)
            stateNormalizer_ = std::unique_ptr<StateNormalizer>(new VectorStateNormalizer());
    }

    virtual ~VectorStateSpace() {

    }

    virtual SpaceType::StateSpaceType getType() const override {
        return SpaceType::VECTORSPACE;
    }

    virtual RobotStateSharedPtr sampleUniform(std::shared_ptr<std::default_random_engine> randGen) const override {
        VectorFloat randomStateVec(dimensions_);
        VectorFloat lowerStateLimits;
        VectorFloat upperStateLimits;
        stateLimits_->getLimits()->as<VectorLimitsContainer>()->get(lowerStateLimits, upperStateLimits);
        for (size_t i = 0; i < dimensions_; i++) {
            std::uniform_real_distribution<FloatType> uniform_dist(lowerStateLimits[i], upperStateLimits[i]);
            FloatType rand_num = uniform_dist(*(randGen.get()));
            randomStateVec[i] = rand_num;
        }

        return std::make_shared<oppt::VectorState>(randomStateVec);
    }

    virtual void makeStateLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits) {
        LimitsContainerSharedPtr container = std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);
        LimitsContainerSharedPtr denormalizedLimitsContainer =
            std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);
        denormalizedStateLimits_ = std::make_shared<VectorStateLimits>(denormalizedLimitsContainer);
        if (stateSpaceInfo_.normalized) {
            stateLimits_ = std::make_shared<NormalizedVectorStateLimits>(container);
        } else {
            stateLimits_ = std::make_shared<VectorStateLimits>(container);
        }

        if (stateNormalizer_)
            stateNormalizer_->setStateLimits(stateLimits_);
    }

    virtual VectorFloat getOrigin() const override {
        if (stateSpaceInfo_.normalized) {
            VectorFloat origin(dimensions_);
            VectorFloat lowerStateLimits;
            VectorFloat upperStateLimits;
            denormalizedStateLimits_->getLimits()->as<VectorLimitsContainer>()->get(lowerStateLimits, upperStateLimits);
            for (size_t i = 0; i < dimensions_; i++) {
                origin[i] = -(lowerStateLimits[i] / (upperStateLimits[i] - lowerStateLimits[i]));
            }

            return origin;
        }

        return VectorFloat(dimensions_, 0);
    }

    virtual RobotStateSharedPtr interpolate(const RobotStateSharedPtr& state1,
                                            const RobotStateSharedPtr& state2,
                                            const FloatType& t) const override {
        VectorFloat state1Vec = static_cast<const VectorState*>(state1.get())->asVector();
        VectorFloat state2Vec = static_cast<const VectorState*>(state2.get())->asVector();
        Vectordf s1 = toEigenVec(state1Vec);
        Vectordf s2 = toEigenVec(state2Vec);
        Vectordf res = s1 + (t * (s2 - s1));
        VectorFloat resVec = toStdVec<FloatType>(res);
        RobotStateSharedPtr resState(new VectorState(resVec));
        return resState;

    }
};

class DiscreteStateSpace: public StateSpace
{
public:
    DiscreteStateSpace(const DiscreteStateSpaceInfo& discreteStateSpaceInfo):
        StateSpace(discreteStateSpaceInfo),
        numDiscreteStates_(discreteStateSpaceInfo.numDiscreteStates) {

    }

    virtual ~DiscreteStateSpace() = default;

    virtual SpaceType::StateSpaceType getType() const override {
        return SpaceType::DISCRETE;
    }

    virtual RobotStateSharedPtr interpolate(const RobotStateSharedPtr& state1,
                                            const RobotStateSharedPtr& state2,
                                            const FloatType& t) const override {
        return nullptr;

    }

    virtual RobotStateSharedPtr sampleUniform(std::shared_ptr<std::default_random_engine> randGen) const override {
        return nullptr;
    }

    virtual VectorFloat getOrigin() const override {
        if (stateSpaceInfo_.normalized) {
            return VectorFloat(dimensions_, 0.5);
        }

        return VectorFloat(dimensions_, 0);
    }

private:
    unsigned int numDiscreteStates_;

};

}

#endif

