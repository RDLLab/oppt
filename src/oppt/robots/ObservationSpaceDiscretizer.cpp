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
#include "oppt/robotHeaders/ObservationSpaceDiscretizer.hpp"
#include "oppt/robotHeaders/ObservationSpace.hpp"

namespace oppt
{
ObservationSpaceDiscretizer::ObservationSpaceDiscretizer(ObservationSpaceSharedPtr& observationSpace):
    observationSpace_(observationSpace)
{

}

std::vector<ObservationSharedPtr> ObservationSpaceDiscretizer::getAllObservationsInOrder(const unsigned int& numStepsPerDimension) const
{
    ObservationLimitsSharedPtr observationLimits = observationSpace_->getObservationLimits();
    VectorFloat lowerLimits;
    VectorFloat upperLimits;
    unsigned int numDimensions = observationSpace_->getNumDimensions();
    if (!observationLimits) {
        ERROR("observation limits is null");
    }

    std::vector<ObservationSharedPtr> allObservationsOrdered_ =
        std::vector<ObservationSharedPtr>(std::pow(numStepsPerDimension, numDimensions));
    observationLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerLimits, upperLimits);
    for (long code = 0; code < std::pow(numStepsPerDimension, numDimensions); code++) {
        VectorFloat ks;
        VectorFloat ss;
        for (size_t i = 0; i < lowerLimits.size(); i++) {
            ks.push_back((upperLimits[i] - lowerLimits[i]) / (numStepsPerDimension - 1));
        }

        FloatType j = code;
        FloatType j_old = code;
        FloatType s = 0;
        for (size_t i = lowerLimits.size() - 1; i != (size_t) - 0; i--) {
            FloatType s;
            j = j_old / std::pow(numStepsPerDimension, i);
            modf(j, &s);
            ss.push_back(s);
            if (i != 1) {
                j = (int)(j_old) % (int)std::pow(numStepsPerDimension, i);
                j_old = j;
            }
        }

        ss.push_back((int)j_old % numStepsPerDimension);
        VectorFloat observationValues;
        for (size_t i = 0; i < lowerLimits.size(); i++) {
            observationValues.push_back(lowerLimits[i] + ss[i] * ks[i]);
        }

        ObservationSharedPtr observation(new DiscreteVectorObservation(observationValues));	
        observation->as<DiscreteVectorObservation>()->setBinNumber(code);
        allObservationsOrdered_[code] = observation;
    }

    return allObservationsOrdered_;
}

}
