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
#include "ParticleFilter.hpp"
#include "oppt/opptCore/ObservationReport.hpp"

namespace oppt
{
Particle::Particle(const RobotStateSharedPtr& state):
    state_(state),
    weight_(1)
{

}

Particle::Particle(const RobotStateSharedPtr& state, const FloatType& weight):
    state_(state),
    weight_(weight)
{

}

const FloatType Particle::getWeight() const
{
    return weight_;
}

void Particle::setWeight(const FloatType& weight)
{
    weight_ = weight;
}

const RobotStateSharedPtr Particle::getState() const
{
    return state_;
}

ParticleSet::ParticleSet():
    particles_(),
    weightedParticleSampler_()
{

}


ParticleSet::ParticleSet(const FilterRequest* filterRequest):
    particles_(filterRequest->previousParticles),
    weightedParticleSampler_()
{
    // Make sure the weights are greater than 0;
    FloatType sumWeights = 0;
    for (size_t i = 0; i != filterRequest->previousParticles.size(); i++) {
        sumWeights += filterRequest->previousParticles[i]->getWeight();
    }

    if (sumWeights == 0) {
        sumWeights = 1.0 / (FloatType)(filterRequest->previousParticles.size());
        for (size_t i = 0; i != particles_.size(); i++) {
            particles_[i]->setWeight(sumWeights);
        }
    }
}

void ParticleSet::setParticles(const VectorParticles& particles)
{
    particles_ = particles;
}


const VectorParticles ParticleSet::sampleUniform(RandomEnginePtr& randomEngine,
        const unsigned int& numSamples) const
{
    std::uniform_int_distribution<long> uniformDist(0, particles_.size() - 1);
    VectorParticles res(numSamples);
    long index = 0;
    for (size_t i = 0; i != numSamples; ++i) {
        index = uniformDist(*(randomEngine.get()));
        res[i] = particles_[index];
    }

    return res;
}

const VectorParticles ParticleSet::sampleWeighted(RandomEnginePtr& randomEngine,
        const unsigned int& numSamples) const
{
    VectorFloat weights(particles_.size());
    for (size_t i = 0; i != particles_.size(); ++i) {
        weights[i] = particles_[i]->getWeight();
    }

    VectorUInt indices =
        weightedParticleSampler_.sample(randomEngine.get(), weights, numSamples);
    VectorParticles res(numSamples);
    for (size_t i = 0; i != numSamples; ++i) {
        res[i] = particles_[indices[i]];
    }

    return res;
}

ParticleFilter::ParticleFilter():
    Filter()
{

}

FilterResultPtr ParticleFilter::propagateParticles(const FilterRequestPtr& filterRequest)
{
    long currNumNextParticles = filterRequest->currentNextParticles.size();
    int deficit = filterRequest->numParticles - currNumNextParticles;
    if (deficit <= 0) {
        FilterResultPtr filterResult = std::make_unique<FilterResult>();
        filterResult->particles = filterRequest->currentNextParticles;        
        return filterResult;
    }

    ParticleSet particleSet(filterRequest.get());
    VectorParticles replenishedParticlesVec(deficit, nullptr);

    unsigned int particleCounter = 0;
    PropagationRequestSharedPtr propagationRequest;
    auto robot = filterRequest->robotEnvironment->getRobot();    
    if (deficit > 0) {
        auto sampledParticles = particleSet.sampleWeighted(filterRequest->randomEngine, deficit);
        for (size_t i = particleCounter; i != deficit; ++i) {
            propagationRequest = std::make_shared<PropagationRequest>();
            propagationRequest->currentState = sampledParticles[i]->getState();
            propagationRequest->action = filterRequest->action;
            propagationRequest->allowCollisions = filterRequest->allowCollisions;
            PropagationResultSharedPtr propagationRes = robot->propagateState(propagationRequest);            

            //auto propagationRes2 = propagationFn_->operator()(propagationRequest);
            bool collided = false;
            if (propagationRes->collisionReport)
                collided = propagationRes->collisionReport->collides;
            if (propagationRequest->allowCollisions || !collided || filterRequest->allowTerminalStates) {
                auto propagatedParticle = std::make_shared<Particle>(propagationRes->nextState, sampledParticles[i]->getWeight());
                // Update the resulting particle weight according to the observation model p(y | x)
                FloatType newWeight = sampledParticles[i]->getWeight();
                if (propagatedParticle->getWeight() > 0 || filterRequest->allowZeroWeightParticles) {
                    replenishedParticlesVec[particleCounter] = propagatedParticle;
                    OpptUserDataSharedPtr userData = replenishedParticlesVec[particleCounter]->getState()->getUserData();                    
                    if (userData) {
                        userData->as<RobotStateUserData>()->previousState = sampledParticles[i]->getState();
                    } else {
                        userData = std::make_shared<RobotStateUserData>();
                        userData->as<RobotStateUserData>()->previousState = sampledParticles[i]->getState();
                        replenishedParticlesVec[particleCounter]->getState()->setUserData(userData);
                    }

                    particleCounter++;
                }
            }            
        }
    }

    

    FilterResultPtr filterResult = std::make_unique<FilterResult>();
    filterResult->particles.resize(currNumNextParticles + particleCounter);
    for (size_t i = 0; i != currNumNextParticles; ++i) {
        filterResult->particles[i] = filterRequest->currentNextParticles[i];
    }
    for (size_t i = 0; i != particleCounter; ++i) {
        filterResult->particles[currNumNextParticles + i] = replenishedParticlesVec[i];
    }

    return std::move(filterResult);
}

FilterResultPtr ParticleFilter::filter(const FilterRequestPtr& filterRequest)
{
    auto robot = filterRequest->robotEnvironment->getRobot();
    long currNumNextParticles = filterRequest->currentNextParticles.size();
    FilterResultPtr filterResult = propagateParticles(filterRequest);
    VectorParticles replenishedParticlesVec(filterResult->particles.size());
    if (replenishedParticlesVec.empty()) {
        WARNING("Couldn't generate any valid particles in ParticleFilter. Check your TerminalPlugin");
    }
    FloatType weightNormalizationConstant = 0.0;
    size_t particleCounter = 0;

    for (size_t i = 0; i != filterResult->particles.size(); ++i) {
        FloatType newWeight = filterResult->particles[i]->getWeight();
        FloatType pdf = 1.0;
        if (filterRequest->observation) {
            pdf = robot->calcLikelihood(filterResult->particles[i]->getState(),
                                        filterRequest->action,
                                        filterRequest->observation);
        } else {
            WARNING("No observation");
        }

        newWeight *= pdf;
        if (newWeight > 0 || filterRequest->allowZeroWeightParticles) {
            replenishedParticlesVec[particleCounter] = filterResult->particles[i];
            replenishedParticlesVec[particleCounter]->setWeight(newWeight);
            weightNormalizationConstant += newWeight;
            particleCounter++;
        } 
    }

    if (particleCounter == 0) {
        // We couldn't sample any particles
        filterResult->particles.resize(0);
        WARNING("All subsequent particles have 0 weight. Check our calcLikelihood function in your ObservationPlugin");
        return filterResult;
    }

    replenishedParticlesVec.resize(particleCounter);

    // Normalize the weights
    if (weightNormalizationConstant == 0) {
        for (auto & elem : replenishedParticlesVec) {
            elem->setWeight(1.0);
        }
    } else {
        for (auto & elem : replenishedParticlesVec) {
            elem->setWeight(elem->getWeight() / weightNormalizationConstant);
        }
    }

    ParticleSet replenishedParticles;
    replenishedParticles.setParticles(replenishedParticlesVec);

    // Resampling step
    filterResult->particles.resize(filterRequest->numParticles);
    auto resampledParticles = replenishedParticles.sampleWeighted(filterRequest->randomEngine, filterRequest->numParticles);
    FloatType newWeight = 1.0 / ((FloatType)resampledParticles.size());    
    for (size_t i = 0; i != resampledParticles.size(); ++i) {
        filterResult->particles[i] = resampledParticles[i];
        filterResult->particles[i]->setWeight(newWeight);        
    }    

    return std::move(filterResult);
}

}
