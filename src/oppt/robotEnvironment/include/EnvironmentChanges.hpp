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
#ifndef __ROBOT_ENVIRONMENT_CHANGES_HPP__
#define __ROBOT_ENVIRONMENT_CHANGES_HPP__
#include "oppt/opptCore/core.hpp"
#include <queue>

namespace oppt
{

enum EnvironmentChangeType {
    MODEL_ADDED,
    MODEL_REMOVED,
    BODY_ADDED,
    BODY_REMOVED,
    BODY_POSE_CHANGED
};

/**
 * An abstract class representing a change in the environment. Classes that implement this class
 * have to implement the getType method
 */
class EnvironmentChange
{
public:
    _NO_COPY_BUT_MOVE(EnvironmentChange)
    EnvironmentChange(const bool &requiresCallback):
        requiresCallback_(requiresCallback) {

    }

    virtual ~EnvironmentChange() = default;

    _STATIC_CAST

    /**
     * @brief Returns true if this EnvironmentChange has to be applied to the underlying model of the GazeboInterface.
     * When the user changes the environment using the Gazebo client, this will return false
     */
    bool requiresCallback() const {
        return requiresCallback_;
    }

    /**
     * @brief Get the type of the EnvironmentChange
     */
    virtual EnvironmentChangeType getType() const = 0;

    /**
     * @brief Serialize the EnvironmentChange
     *
     * @param step The step in which the change occured
     * @param os Stream to the output file
     */
    virtual void serialize(std::ofstream& os) const = 0;

    /** @brief Print to an output stream */
    virtual void print(std::ostream& os) const {}

    friend std::ostream& operator<< (std::ostream& out, const EnvironmentChange& environmentChange) {
        environmentChange.print(out);
        return out;
    }

private:
    bool requiresCallback_;
};

class ModelAddedChange: public EnvironmentChange {
public:
    using EnvironmentChange::EnvironmentChange;
    _NO_COPY_BUT_MOVE(ModelAddedChange)
    ModelAddedChange(const std::string &sdfString, const bool &requiresCallback):
        EnvironmentChange(requiresCallback),
        sdfString_(sdfString) {

    }

    virtual ~ModelAddedChange() = default;

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::MODEL_ADDED;
    }

    const std::string getSDFString() const {
        return sdfString_;
    }

    virtual void serialize(std::ofstream &os) const override {
        std::string sdfTrimmed = sdfString_;

        // Remove \n characters
        sdfTrimmed.erase(std::remove(sdfTrimmed.begin(), sdfTrimmed.end(), '\n'), sdfTrimmed.end());
        os << "add " << sdfTrimmed << endl;
    }

private:
    const std::string sdfString_;

};

class ModelRemovedChange: public EnvironmentChange {
public:
    using EnvironmentChange::EnvironmentChange;
    _NO_COPY_BUT_MOVE(ModelRemovedChange)
    ModelRemovedChange(const std::string& modelName, const bool &requiresCallback = false):
        EnvironmentChange(requiresCallback),
        modelName_(modelName) {

    }

    virtual ~ModelRemovedChange() = default;

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::MODEL_REMOVED;
    }

    const std::string getModelName() const {
        return modelName_;
    }

    virtual void serialize(std::ofstream& os) const override {
        os << "remove " << modelName_ << endl;
    }

    virtual void print(std::ostream& os) const override {
        os << "remove" << modelName_;
    }   

private:
    const std::string modelName_;
};

/**
 * Represents a change in the environment when an body has been added
 */
class BodyAddedChange: public EnvironmentChange
{
public:
    using EnvironmentChange::EnvironmentChange;
    _NO_COPY_BUT_MOVE(BodyAddedChange)
    BodyAddedChange(const std::string& sdfString, const bool &requiresCallback = false):
        EnvironmentChange(requiresCallback),
        sdfString_(sdfString) {

    }

    virtual ~BodyAddedChange() = default;

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::BODY_ADDED;
    }

    const std::string getSDFString() const {
        return sdfString_;
    }

    virtual void serialize(std::ofstream& os) const override {
        std::string sdfTrimmed = sdfString_;

        // Remove \n characters
        sdfTrimmed.erase(std::remove(sdfTrimmed.begin(), sdfTrimmed.end(), '\n'), sdfTrimmed.end());
        os << "add " << sdfTrimmed << endl;
    }

private:
    const std::string sdfString_;
};

/**
 * Represents a change in the environment when an body has been removed
 */
class BodyRemovedChange: public EnvironmentChange
{
public:
    using EnvironmentChange::EnvironmentChange;
    _NO_COPY_BUT_MOVE(BodyRemovedChange)
    BodyRemovedChange(const std::string& bodyName, const bool &requiresCallback = false):
        EnvironmentChange(requiresCallback),
        bodyName_(bodyName) {

    }

    virtual ~BodyRemovedChange() = default;

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::BODY_REMOVED;
    }

    const std::string getBodyName() const {
        return bodyName_;
    }

    virtual void serialize(std::ofstream& os) const override {
        os << "remove " << bodyName_ << endl;
    }

    virtual void print(std::ostream& os) const override {
        os << "remove" << bodyName_;
    }

private:
    const std::string bodyName_;

};

/**
 * Represents a change in the environment when the pose of an body has changed
 */
class BodyPoseChange: public EnvironmentChange
{
public:
    using EnvironmentChange::EnvironmentChange;
    _NO_COPY_BUT_MOVE(BodyPoseChange)
    BodyPoseChange(const std::string& bodyName,
                       const geometric::Pose& pose,
                       const bool &requiresCallback = false):
        EnvironmentChange(requiresCallback),
        bodyName_(bodyName),
        pose_(pose) {

    }

    virtual ~BodyPoseChange() = default;

    const std::string getBodyName() const {
        return bodyName_;
    }

    const geometric::Pose getPose() const {
        return pose_;
    }

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::BODY_POSE_CHANGED;
    }

    virtual void serialize(std::ofstream& os) const override {
        VectorFloat poseVec(6, 0.0);
        Vector3f eulerAngles = math::quaternionToEulerAngles(pose_.orientation);
        poseVec[0] = pose_.position.x();
        poseVec[1] = pose_.position.y();
        poseVec[2] = pose_.position.x();
        poseVec[3] = eulerAngles[0];
        poseVec[4] = eulerAngles[1];
        poseVec[5] = eulerAngles[2];

        os << "changePose " << bodyName_ << " ";
        for (size_t i = 0; i != poseVec.size(); ++i) {
            os << poseVec[i] << " ";
        }

        os << endl;
    }

private:
    const std::string bodyName_;
    const geometric::Pose pose_;

};

/**
 * A simple wrapper class around a queue of environment changes
 */
class EnvironmentChanges
{
public:
    _NO_COPY_BUT_MOVE(EnvironmentChanges)
    EnvironmentChanges() = default;

    ~EnvironmentChanges() = default;

    void addChange(EnvironmentChangeSharedPtr environmentChange) {
        environmentChanges_.push(environmentChange);
    }

    const EnvironmentChangeSharedPtr getNextChange() {
        if (environmentChanges_.size() == 0)
            return nullptr;
        EnvironmentChangeSharedPtr nextChange = environmentChanges_.front();
        environmentChanges_.pop();
        return nextChange;
    }

    const unsigned int getNumChanges() const {
        return environmentChanges_.size();
    }

private:
    std::queue<EnvironmentChangeSharedPtr> environmentChanges_;

};

/** @brief std::shared_ptr to oppt::EnvironmentChanges */
typedef std::shared_ptr<EnvironmentChanges> EnvironmentChangesPtr;
}

#endif
