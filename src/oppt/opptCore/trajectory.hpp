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
#ifndef __OPPT_TRAJECTORY_HPP__
#define __OPPT_TRAJECTORY_HPP__
#include "typedefs.hpp"
#include "Action.hpp"
#include "Observation.hpp"
#include "OpptUserData.hpp"

namespace oppt
{

/**
 * Represents a trajectory. A trajectory consists of (state, action, observation) tuples
 */
class Trajectory: public OpptUserData
{
public:
    /** @brief Default constructor */
    Trajectory() {}
    
    /** @brief Default destructor */
    virtual ~Trajectory() {}

    /** @brief The state trajectory */
    std::vector<RobotStateSharedPtr> stateTrajectory;

    /** @brief The action trajectory */
    std::vector<ActionSharedPtr> actionTrajectory;

    /** @brief The observation trajectory */
    std::vector<ObservationSharedPtr> observationTrajectory;

    /** @brief Contains the durations of each element of the trajectory */
    VectorFloat durations;

    /** @brief Get the size (number of states) of the trajectory */
    unsigned int size() const {
        return stateTrajectory.size();
    }

    /** @brief Copy the trajectory */
    virtual TrajectoryUniquePtr copy() const {
        TrajectoryUniquePtr copiedTrajectory(new Trajectory());
        copiedTrajectory->stateTrajectory = stateTrajectory;
        copiedTrajectory->actionTrajectory = actionTrajectory;
        copiedTrajectory->observationTrajectory = observationTrajectory;
        copiedTrajectory->durations = durations;
        return std::move(copiedTrajectory);
    } 

    friend std::ostream& operator<< (std::ostream& out, const Trajectory& trajectory) {
        for (size_t i = 0; i != trajectory.stateTrajectory.size(); i++) {
            out << *(trajectory.stateTrajectory[i].get()) << "\n";
            if (trajectory.actionTrajectory.empty() == false and trajectory.actionTrajectory[i])
                out << *(trajectory.actionTrajectory[i].get()) << "\n";
            if (trajectory.observationTrajectory.empty() == false and trajectory.observationTrajectory[i])
                out << *(trajectory.observationTrajectory[i].get()) << "\n";
        }

        return out;
    }

};


struct VectorTrajectory {
    VectorTrajectory() {}

    std::vector<VectorFloat> states;

    std::vector<VectorFloat> controls;

    std::vector<VectorFloat> observations;

    VectorFloat durations;

    void print(std::ostream& out) const {
        out << "TRAJECTORY BEGIN\n";
        for (size_t i = 0; i < states.size(); i++) {
            out << "state: ";
            for (size_t j = 0; j < states[i].size(); j++) {
                out << states[i][j] << ", ";
            }
            out << "\n";

            out << "control: ";
            for (size_t j = 0; j < controls[i].size(); j++) {
                out << controls[i][j] << ", ";
            }
            out << "\n";

            out << "observation: ";
            for (size_t j = 0; j < observations[i].size(); j++) {
                out << observations[i][j] << ", ";
            }
            out << "\n";
        }

        out << "TRAJECTORY END\n";
    }
};

}

#endif
