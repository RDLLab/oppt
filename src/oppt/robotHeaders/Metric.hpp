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
#ifndef __OPPT__METRIC_HPP__
#define __OPPT__METRIC_HPP__
#include "oppt/opptCore/core.hpp"

namespace oppt
{

/**
 * Represents a (distance) metric between two RobotStates
 */
struct Metric {
    /**
     * @brief Calculates the metrix between two RobotStates
     * @param state1 The first state
     * @param state2 The second state
     */
    virtual FloatType operator()(const oppt::RobotStateSharedPtr& state1,
                                 const oppt::RobotStateSharedPtr& state2) const = 0;

};

/**
 * The Euclidean distance metric between two RobotStates
 */
struct EuclideanMetric: public Metric {
    virtual FloatType operator()(const oppt::RobotStateSharedPtr& state1,
                                 const oppt::RobotStateSharedPtr& state2) const override {
        const VectorFloat s1Vec =
            static_cast<oppt::VectorState*>(state1.get())->asVector();
        const VectorFloat s2Vec =
            static_cast<oppt::VectorState*>(state2.get())->asVector();
        FloatType sum = 0.0;
        [&](const VectorFloat & vec1, const VectorFloat & vec2, FloatType & sum) {
            for (size_t i = 0; i < vec1.size(); i++) {
                sum += std::pow(vec2[i] - vec1[i], 2);
            }

            sum = std::sqrt(sum);
        }(s1Vec, s2Vec, sum);

        return sum;

    }
};

}

#endif
