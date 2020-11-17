#ifndef MANIPULATOR_TRANSITION_HPP_
#define MANIPULATOR_TRANSITION_HPP_

#include <unistd.h>
#include "solvers/ABT/solver/abstract-problem/TransitionParameters.hpp"
#include "RobotState.hpp"

namespace oppt
{
class ApproximateStepGenerator;
};

namespace shared
{

class RobotTransitionParameters: public abt::TransitionParameters
{
    friend class oppt::ApproximateStepGenerator;

public:
    RobotTransitionParameters(oppt::PropagationResultSharedPtr& propagationResult);

    RobotTransitionParameters(const oppt::PropagationResultSharedPtr& propagationResult);
    
    virtual ~RobotTransitionParameters() {}

    virtual void print(std::ostream& os) const override;

    virtual void serialize(std::ostream& os, const std::string prefix = "") const override;

    oppt::PropagationResultSharedPtr getPropagationResult() const;

    std::unique_ptr<abt::TransitionParameters> copy() const;

    const bool approximateDynamics() const;

private:
    bool approximateDynamics_;

    oppt::PropagationResultSharedPtr propagationResult_;
};

}

#endif
