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
#ifndef __OPPT__SERIALIZER__HPP__
#define __OPPT__SERIALIZER__HPP__
#include "oppt/opptCore/core.hpp"
#include "Action.hpp"
#include "Observation.hpp"
#include "RobotState.hpp"

namespace oppt
{

/**
 * Base class for all serializers
 */
class Serializer {
public:
    Serializer() = default;
    virtual ~Serializer() = default;
    _NO_COPY_BUT_MOVE(Serializer)
    _STATIC_CAST
};

/**
 * A class used for serialization of oppt::OpptUserData objects
 */
class UserDataSerializer: public Serializer {
public:
    UserDataSerializer():
        Serializer() {

    }

    virtual ~UserDataSerializer() = default;

    virtual OpptUserDataSharedPtr loadUserData(std::istream& is) const {
        return nullptr;
    }
};

/** @brief Unique pointer to oppt::UserDataSerializer */
typedef std::unique_ptr<UserDataSerializer> UserDataSerializerPtr;

/**
 * A class used for serialization of states, actions and observations
 */
class RobotObjectSerializer: public Serializer
{
public:
    RobotObjectSerializer():
        Serializer(),
        userDataSerializer_(std::make_unique<UserDataSerializer>()) {
    }

    /**
     * @brief Set the oppt::UserDataSerializerPtr 
     */
    void setUserDataSerializer(UserDataSerializerPtr userDataSerializer) {
        userDataSerializer_ = std::move(userDataSerializer);
    }

    /**
     * @brief Loads a RobotState from an input stream
     * @param is The input stream
     * @returns A shared pointer to the loaded RobotState
     */
    virtual oppt::RobotStateSharedPtr loadState(std::istream& is) const = 0;

    /**
     * @brief Loads a RobotState from an input string
     * @param input The input string
     * @returns A shared pointer to the loaded RobotState
     */
    virtual RobotStateSharedPtr loadState(const std::string& input) const = 0;

    /**
     * @brief Loads an Action from an input stream
     * @param is The input stream
     * @returns A shared pointer to the loaded Action
     */
    virtual oppt::ActionSharedPtr loadAction(std::istream& is) const = 0;

    /**
     * @brief Loads an Observation from an input stream
     * @param is The input stream
     * @returns A shared pointer to the loaded Observation
     */
    virtual oppt::ObservationSharedPtr loadObservation(std::istream& is) const = 0;

    virtual OpptUserDataSharedPtr loadUserData(std::istream& is) const = 0;

    virtual std::vector<oppt::RobotStateSharedPtr> loadGoalStatesFromFile(std::string& filename) const = 0;

protected:
    UserDataSerializerPtr userDataSerializer_ = nullptr;
};

/**
 * Specialization of the Serializer for vector valued states, actions and observations
 */
class VectorSerializer: public RobotObjectSerializer
{
public:
    VectorSerializer();

    virtual oppt::RobotStateSharedPtr loadState(std::istream& is) const override;

    virtual RobotStateSharedPtr loadState(const std::string& input) const override;

    virtual oppt::ActionSharedPtr loadAction(std::istream& is) const override;

    virtual oppt::ObservationSharedPtr loadObservation(std::istream& is) const override;

    virtual std::vector<oppt::RobotStateSharedPtr> loadGoalStatesFromFile(std::string& filename) const override;

};

}

#endif
