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
#ifndef __OPPT_ACTION_HPP__
#define __OPPT_ACTION_HPP__
#include "oppt/opptCore/core.hpp"

namespace oppt
{

/**
 * Specialization of the Action interface for real vector valued continuous actions
 */
class VectorAction: public Action
{
public:
    using Action::Action;

    _NO_COPY_BUT_MOVE(VectorAction)
    /**
     * @brief Construct from a VectorFloat
     */
    VectorAction(VectorFloat& actionValues);

    /**
     * @brief Construct from a const VectorFloat
     */
    VectorAction(const VectorFloat& actionValues);

    virtual ~VectorAction() = default;

    virtual oppt::ActionUniquePtr copy() const override;

    virtual void print(std::ostream& os) const override;

    virtual void serialize(std::ostream& os, const std::string& prefix = "") const override;

    virtual bool equals(const Action& otherAction) const override;

    virtual std::size_t hash() const override;

    virtual FloatType distanceTo(const Action& otherAction) const override;

    virtual FloatType distanceTo(const oppt::ActionSharedPtr& action) const override;

    /**
     * @brief Get the underlying action vector
     * @return A VectorFloat with the underlying raw action values
     */
    virtual VectorFloat asVector() const;

protected:
    VectorFloat actionVec_;

    long binNumber_;

};

/**
 * Specialization of a VectorAction for discrete real vector actions. The discretization is
 * represented by a bin number. Two actions with the same bin number are considered equal
 */
class DiscreteVectorAction: public VectorAction
{

public:
    using VectorAction::VectorAction;
    _NO_COPY_BUT_MOVE(DiscreteVectorAction)
    /**
     * @brief Construct from a VectorFloat
     */
    DiscreteVectorAction(VectorFloat& actionValues);

    /**
     * @brief Construct from a const VectorFloat
     */
    DiscreteVectorAction(const VectorFloat& actionValues);

    /**
     * @brief Get the discrete bin number for this action
     */
    long getBinNumber() const;

    /**
     * @brief Set the discrete bin number for this action
     */
    void setBinNumber(long binNumber);

};

}

#endif
