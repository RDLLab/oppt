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
#ifndef __OPPT_UTILS_SAMPLER_HPP__
#define __OPPT_UTILS_SAMPLER_HPP__
#include "oppt/opptCore/core.hpp"
#include "oppt/global.hpp"

namespace oppt
{

class Sampler
{
public:
    Sampler();

    template<class T>
    T* as() {
        return static_cast<T*>(this);
    }

    virtual void** sample(RandomGenerator* randGen,
                          void** sampleSet,
                          const unsigned int& sampleSize,
                          const unsigned int& numSamples) = 0;

protected:
    std::vector<void*> tempSamples2_;
};

class UniformDiscreteSampler: public Sampler
{
public:
    UniformDiscreteSampler();

    virtual void** sample(RandomGenerator* randGen,
                          void** sampleSet,
                          const unsigned int& sampleSize,
                          const unsigned int& numSamples) override;

};

class WeightedDiscreteSampler: public Sampler
{
public:
    WeightedDiscreteSampler();

    virtual void** sample(RandomGenerator* randGen,
                          void** sampleSet,
                          const unsigned int& sampleSize,
                          const unsigned int& numSamples) override;

    std::vector<unsigned int> sample(RandomGenerator* randGen, 
				     const VectorFloat &weights,
				     const unsigned int& numSamples) const;

private:
    VectorFloat weights_;

    FloatType weightSum_;

};

}

#endif
