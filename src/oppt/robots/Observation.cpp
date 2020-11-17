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
#include "oppt/robotHeaders/Observation.hpp"
#include "oppt/global.hpp"

namespace oppt
{

VectorObservation::VectorObservation(VectorFloat& observationVec):
    Observation(),
    observationVec_(observationVec),
    binNumber_(0)
{

}

VectorObservation::VectorObservation(const VectorFloat& observationVec):
    Observation(),
    observationVec_(observationVec),
    binNumber_(0)
{

}

void VectorObservation::serialize(std::ostream& os, const std::string& prefix) const
{
    if (prefix != "") {
        os << prefix << ": ";
    }
    for (auto & k : observationVec_) {
        os << k << " ";
    }

    os << "END ";
}

void VectorObservation::print(std::ostream& os) const
{
    for (auto & k : observationVec_) {
        os << k << " ";
    }
}

bool VectorObservation::equals(const Observation& otherObservation) const
{
    VectorFloat otherObservationVec =
        static_cast<const VectorObservation&>(otherObservation).asVector();
    for (size_t i = 0; i < otherObservationVec.size(); i++) {
	if (std::fabs(observationVec_[i] - otherObservationVec[i]) > 1e-7)
	    return false;
    }

    return true;
}

std::size_t VectorObservation::hash() const
{
    size_t hashValue = 0;
    for (auto & k : observationVec_) {
        oppt::hash_combine(hashValue, k);
    }

    return hashValue;
}

FloatType VectorObservation::distanceTo(const Observation& otherObservation) const
{
    VectorFloat otherObservationVec =
        static_cast<const VectorObservation&>(otherObservation).asVector();
    return math::euclideanDistance(observationVec_, otherObservationVec);    
    /**FloatType distance = 0.0;
    for (size_t i = 0; i < otherObservationVec.size(); i++) {
        distance += std::pow(observationVec_[i] - otherObservationVec[i], 2);
    }

    return std::sqrt(distance);*/
}

VectorFloat VectorObservation::asVector() const
{
    return observationVec_;
}

DiscreteVectorObservation::DiscreteVectorObservation(VectorFloat& observationValues):
    VectorObservation(observationValues)
{

}

DiscreteVectorObservation::DiscreteVectorObservation(const VectorFloat& observationValues):
    VectorObservation(observationValues)
{

}

bool DiscreteVectorObservation::equals(const Observation& otherObservation) const {
    return static_cast<const DiscreteVectorObservation *>(&otherObservation)->getBinNumber() == binNumber_;
}

long DiscreteVectorObservation::getBinNumber() const
{   
    return binNumber_;
}

void DiscreteVectorObservation::setBinNumber(long binNumber)
{    
    binNumber_ = binNumber;
}

}
