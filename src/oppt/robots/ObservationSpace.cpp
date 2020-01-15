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
#include "oppt/robotHeaders/ObservationSpace.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace oppt
{

ObservationSpace::ObservationSpace(const ObservationSpaceInfo& observationSpaceInfo):
    dimension_(observationSpaceInfo.numDimensions),
    observationSpaceInfo_(observationSpaceInfo),
    observationNormalizer_(nullptr),
    observationLimits_(nullptr),
    denormalizedObservationLimits_(nullptr)
{
    
}

ObservationLimitsSharedPtr ObservationSpace::getObservationLimits() const
{
    return observationLimits_;
}

size_t ObservationSpace::getNumDimensions() const
{
    return dimension_;
}

const ObservationSpaceInfo ObservationSpace::getInfo() const
{
    return observationSpaceInfo_;
}

bool ObservationSpace::enforceObservationLimits(ObservationSharedPtr& observation) const
{
    if (!observationLimits_) {
        WARNING("Trying to enforce observation limits, but no limits set");
        return false;

    } else {
        return observationLimits_->enforceLimits(observation);
    }
}

void ObservationSpace::setObservationNormalizer(std::unique_ptr<ObservationNormalizer> observationNormalizer) {
    observationNormalizer_ = std::move(observationNormalizer);
}

ObservationSharedPtr VectorObservationNormalizer::normalizeObservation(const ObservationSharedPtr& observation) const {
    VectorFloat lowerObservationLimits;
    VectorFloat upperObservationLimits;
    observationLimits_->getRawLimits()->as<VectorLimitsContainer>()->get(lowerObservationLimits, upperObservationLimits);
    VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
    for (size_t i = 0; i < observationVec.size(); i++) {
        observationVec[i] =
            (observationVec[i] - lowerObservationLimits[i]) / (upperObservationLimits[i] - lowerObservationLimits[i]);
    }

    return std::make_shared<VectorObservation>(observationVec);
}

ObservationSharedPtr VectorObservationNormalizer::denormalizeObservation(const ObservationSharedPtr& observation) const {
    VectorFloat lowerObservationLimits;
    VectorFloat upperObservationLimits;
    observationLimits_->getRawLimits()->as<VectorLimitsContainer>()->get(lowerObservationLimits, upperObservationLimits);
    VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
    for (size_t i = 0; i < observationVec.size(); i++) {
        observationVec[i] = observationVec[i] * (upperObservationLimits[i] - lowerObservationLimits[i]) + lowerObservationLimits[i];
    }

    return std::make_shared<VectorObservation>(observationVec);
}

VectorObservationSpace::VectorObservationSpace(const ObservationSpaceInfo& observationSpaceInfo):
    ObservationSpace(observationSpaceInfo)
{
    if (observationSpaceInfo_.normalized)
        observationNormalizer_ = std::unique_ptr<ObservationNormalizer>(new VectorObservationNormalizer());
}

SpaceType::StateSpaceType VectorObservationSpace::getType() const
{
    return SpaceType::VECTORSPACE;
}

VectorFloat VectorObservationSpace::getOrigin() const
{
    if (observationSpaceInfo_.normalized) {
        VectorFloat origin(dimension_);
        VectorFloat lowerObservationLimits;
        VectorFloat upperObservationLimits;
        denormalizedObservationLimits_->getLimits()->as<VectorLimitsContainer>()->get(lowerObservationLimits,
                upperObservationLimits);
        for (size_t i = 0; i < dimension_; i++) {
            origin[i] = -(lowerObservationLimits[i] / (upperObservationLimits[i] - lowerObservationLimits[i]));
        }

        return origin;
    }

    return VectorFloat(dimension_, 0);
}

void VectorObservationSpace::makeObservationLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits)
{
    LimitsContainerSharedPtr container = std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);
    LimitsContainerSharedPtr denormalizedLimitsContainer = std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);
    denormalizedObservationLimits_ = std::make_shared<VectorObservationLimits>(denormalizedLimitsContainer);
    if (observationSpaceInfo_.normalized) {
        observationLimits_ = std::make_shared<NormalizedVectorObservationLimits>(container);
    } else {
        observationLimits_ = std::make_shared<VectorObservationLimits>(container);
    }

    if (observationNormalizer_)
        observationNormalizer_->setObservationLimits(observationLimits_);
}

}
