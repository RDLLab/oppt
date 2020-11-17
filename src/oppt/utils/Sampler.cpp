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
#include "include/Sampler.hpp"

namespace oppt
{

//################### UniformDiscreteSampler ###################
void** UniformDiscreteSampler::sample(RandomGenerator* randGen,
                                      void** sampleSet,
                                      const unsigned int& sampleSize,
                                      const unsigned int& numSamples)
{
    std::uniform_real_distribution<FloatType> uniformDist(0, sampleSize);
    tempSamples2_.resize(numSamples);
    size_t index;
    for (size_t i = 0; i != numSamples; ++i) {
        index = uniformDist(*(randGen));
        tempSamples2_[i] = sampleSet[index];
    }

    return tempSamples2_.data();
}

//################### WeightedDiscreteSampler ###################
void** WeightedDiscreteSampler::sample(RandomGenerator* randGen,
                                       void** sampleSet,
                                       const unsigned int& sampleSize,
                                       const unsigned int& numSamples)
{
    VectorFloat weights(sampleSize);
    FloatType sumWeights = 0;
    for (size_t i = 0; i != sampleSize; ++i) {
        FloatType weight = static_cast<const std::pair<const void*, FloatType> *>(sampleSet[i])->second;
        sumWeights += weight;
        weights[i] = weight;
    }

    std::uniform_real_distribution<FloatType> uniformDist(0, sumWeights);
    tempSamples2_.resize(numSamples);
    for (size_t i = 0; i != numSamples; ++i) {
        FloatType randNum = uniformDist(*(randGen));
        long index = 0;
        for (size_t j = 0; j != weights.size(); ++j) {
            if (randNum < weights[j]) {
                index = j;
                break;
            }
            randNum -= weights[j];
        }

        tempSamples2_[i] = sampleSet[index];
    }

    return tempSamples2_.data();
}

VectorUInt WeightedDiscreteSampler::sample(RandomGenerator* randGen,
        const VectorFloat &weights,        
        const unsigned int& numSamples) const
{    
    
    std::discrete_distribution<unsigned int> d(weights.begin(), weights.end());
    std::vector<unsigned int> sampledIndices(numSamples);
    for (size_t i = 0; i != numSamples; ++i) {
	sampledIndices[i] = d(*randGen);
    }
    
    return sampledIndices;
}

}
