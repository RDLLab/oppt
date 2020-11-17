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
#ifndef __ACT_SPACE_HPP__
#define __ACT_SPACE_HPP__
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include "oppt/opptCore/core.hpp"
#include "Action.hpp"
#include "LimitsContainer.hpp"
#include "ActionSpaceDiscretizer.hpp"
#include "oppt/opptCore/SpaceComponents.hpp"

using std::cout;
using std::endl;

namespace oppt
{

struct ActionSpaceInfo {
    ActionSpaceInfo() {}

    bool normalized = false;

    size_t numDimensions = 1;

    SpaceVariablesPtr spaceVariables;
};

class ActionLimits
{    
public:
    ActionLimits(const LimitsContainerSharedPtr& limits):
        container_(limits) {

    }

    virtual ~ActionLimits() {
        container_.reset();
    }

    template<class T>
    T* as() {
        return static_cast<T*>(this);
    }

    virtual bool enforceLimits(oppt::ActionSharedPtr& action) const = 0;

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

class VectorActionLimits: public ActionLimits
{
public:
    VectorActionLimits(const LimitsContainerSharedPtr& limits):
        ActionLimits(limits),
        lowerLimits_(),
        upperLimits_() {
        container_->as<VectorLimitsContainer>()->get(lowerLimits_, upperLimits_);
    }

    virtual ~VectorActionLimits() {
        lowerLimits_.clear();
        upperLimits_.clear();
    }

    virtual bool enforceLimits(oppt::ActionSharedPtr& action) const override {
        VectorFloat actionVec =
            static_cast<oppt::VectorAction*>(action.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < actionVec.size(); i++) {
            if (actionVec[i] < lowerLimits_[i]) {
                enforced = true;
                actionVec[i] = lowerLimits_[i];
            } else if (actionVec[i] > upperLimits_[i]) {
                enforced = true;
                actionVec[i] = upperLimits_[i];
            }
        }

        action = std::make_shared<oppt::VectorAction>(actionVec);
        return enforced;
    }

protected:
    VectorFloat lowerLimits_;
    VectorFloat upperLimits_;
};

class NormalizedVectorActionLimits: public VectorActionLimits
{
public:
    NormalizedVectorActionLimits(const LimitsContainerSharedPtr& limits):
        VectorActionLimits(limits),
        normalizedLimitsContainer_(nullptr) {
        VectorFloat lowerLimits(lowerLimits_.size(), 0);
        VectorFloat upperLimits(upperLimits_.size(), 1);
        normalizedLimitsContainer_ =
            std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);

    }

    virtual ~NormalizedVectorActionLimits() {
        normalizedLimitsContainer_.reset();
    }

    virtual bool enforceLimits(oppt::ActionSharedPtr& action) const override {
        VectorFloat actionVec =
            static_cast<oppt::VectorAction*>(action.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < actionVec.size(); i++) {
            if (actionVec[i] < 0) {
                enforced = true;
                actionVec[i] = 0;
            } else if (actionVec[i] > 1) {
                enforced = true;
                actionVec[i] = 1;
            }
        }

        action = std::make_shared<oppt::VectorAction>(actionVec);
        return enforced;
    }

    virtual const LimitsContainerSharedPtr& getLimits() const override {
        return normalizedLimitsContainer_;
    }

private:
    LimitsContainerSharedPtr normalizedLimitsContainer_;
};

/**
 * A class to normalize and denormalize actions
 */
class ActionNormalizer
{
public:
    virtual ~ActionNormalizer()=default;
    /**
     * @brief Normalize an Action
     */    
    virtual ActionSharedPtr normalizeAction(const ActionSharedPtr& action) const = 0;

    /**
     * @brief Deormalize an Action
     */    
    virtual ActionSharedPtr denormalizeAction(const ActionSharedPtr& action) const = 0;

    void setActionLimits(oppt::ActionLimitsSharedPtr& actionLimits) {
        actionLimits_ = actionLimits;
    }

protected:
    oppt::ActionLimitsSharedPtr actionLimits_ = nullptr;

};

/**
 * Represents a POMDP action space
 */
class ActionSpace
{
public:
    /**
     * @brief Construct from ActionSpaceInfo
     */
    ActionSpace(const ActionSpaceInfo& actionSpaceInfo);

    virtual ~ActionSpace() {
        actionLimits_.reset();
        denormalizedActionLimits_.reset();
        actionSpaceInformation_.reset();        
    }

    _NO_COPY_BUT_MOVE(ActionSpace)

    _STATIC_CAST

    /**
     * @brief Returns the type of the action space
     */
    virtual std::string getType() const = 0;

    /**
     * @brief Sample an Action uniformly at random
     */
    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const = 0;

    /**
     * @brief If the provided Action is out of the action limits, force it into the limits
     */
    bool enforceActionLimits(ActionSharedPtr& action) const;
    
    ActionLimitsSharedPtr getActionLimits() const;

    /**
     * @brief Get the number of dimensions
     */
    size_t getNumDimensions() const;

    /**
     * @brief Get the ActionSpaceInfo this action space was constructed from
     */
    const ActionSpaceInfo getInfo() const;

    virtual VectorFloat getOrigin() const = 0;

    /**
     * @brief Normalizes an action to [0, 1]
     * If the oppt::ProblemEnvironmentOptions::normalizedSpaces is false, this method has no effect     
     * @param observation The action to normalize
     * @return A shared pointer to the normalized action
     */    
    ActionSharedPtr normalizeAction(const ActionSharedPtr& action) const;

    /**
     * @brief Denormalizes an action
     * If the oppt::ProblemEnvironmentOptions::normalizedSpaces is false, this method has no effect     
     * @param observation The action to denormalize
     * @return A oppt::ActionSharedPtr to the denormalized action
     */    
    ActionSharedPtr denormalizeAction(const ActionSharedPtr& action) const;

    /**
     * @brief Get the action space discretizer. If no action space discretizer was set using
     * oppt::ActionSpace::setActionSpaceDiscretizer, a nullptr will be returned
     */
    virtual std::shared_ptr<ActionSpaceDiscretizer> getActionSpaceDiscretizer() const {
        return actionSpaceDiscretizer_;
    }

    /**
     * @brief Set an action space discretizer that discretizes the action space
     */
    virtual void setActionSpaceDiscretizer(std::shared_ptr<ActionSpaceDiscretizer> &actionSpaceDiscretizer) {
        actionSpaceDiscretizer_ = actionSpaceDiscretizer;
    }

    /**
     * @brief Set a custom oppt::ActionNormalizer
     */
    virtual void setActionNormalizer(std::unique_ptr<ActionNormalizer> actionNormalizer);

protected:
    size_t numDimensions_;

    ActionLimitsSharedPtr actionLimits_;

    ActionLimitsSharedPtr denormalizedActionLimits_;

    ActionSpaceInfo actionSpaceInfo_;

    ActionSpaceInformationPtr actionSpaceInformation_;
    
    std::unique_ptr<ActionNormalizer> actionNormalizer_ = nullptr;

    std::shared_ptr<ActionSpaceDiscretizer> actionSpaceDiscretizer_ = nullptr;

};

class ContinuousActionSpace: public ActionSpace
{
public:
    ContinuousActionSpace(const ActionSpaceInfo& actionSpaceInfo);

    virtual ~ContinuousActionSpace() {

    }

    virtual std::string getType() const override;

};

/**
 * Specialization of a opp::ActionNormalizer for vector spaces
 */
class VectorActionNormalizer: public ActionNormalizer {
public:
    virtual ~VectorActionNormalizer() = default;

    virtual ActionSharedPtr normalizeAction(const ActionSharedPtr& action) const override;

    virtual ActionSharedPtr denormalizeAction(const ActionSharedPtr& action) const override;


};

class ContinuousVectorActionSpace: public ContinuousActionSpace
{
public:
    ContinuousVectorActionSpace(const ActionSpaceInfo& actionSpaceInfo);

    virtual ~ContinuousVectorActionSpace() {

    }

    virtual void makeActionLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits);

    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const override;

    virtual VectorFloat getOrigin() const override;
};

}

#endif
