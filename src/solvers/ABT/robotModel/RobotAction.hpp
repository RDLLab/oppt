/** @file RockSampleAction.hpp
 *
 * Defines the RockSampleAction class, which represents an action for the RockSample problem, and
 * also the ActionType enumeration, which enumerates the different types of actions for RockSample.
 */
#ifndef MANIPULATOR_ACTION_HPP_
#define MANIPULATOR_ACTION_HPP_

#include <cstddef>                      // for size_t
#include <cstdint>

#include <ostream>                      // for ostream
#include <vector>                       // for vector


#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint
#include "solvers/ABT/solver/mappings/actions/continuous_actions.hpp"
#include "oppt/robotHeaders/Action.hpp"

using namespace oppt;

namespace robot
{

class ContActionConstructionData final: public abt::ContinuousActionConstructionDataBase
{
public:
    ContActionConstructionData(const FloatType* constructionDataVector, size_t dataSize);

    ContActionConstructionData(oppt::ActionSharedPtr& action);

    ContActionConstructionData(const oppt::ActionSharedPtr& action);

    virtual const FloatType* data() const override;

    size_t size() const;

    FloatType& operator[](size_t index);

    const FloatType& operator[](size_t index) const;

    oppt::ActionSharedPtr getAction() const;

public:
    /* Infrastructure for use in a ContinuousActionContainer */
    struct HashEqualOptions {
        HashEqualOptions(const FloatType theTolerance): tolerance(theTolerance) {}

        /* Defines the tolerance used when hashing and comparing elements. */
        const FloatType tolerance;
    };

    size_t hash(const HashEqualOptions& options) const;

    bool equal(const ContActionConstructionData& other, const HashEqualOptions& options) const;

private:
    FloatType snapToTolerance(const FloatType value, const FloatType tolerance);

    oppt::ActionSharedPtr storage;
    
    VectorFloat vectorData_;
};

class ContinuousRobotAction: public abt::ContinuousAction
{
public:

    typedef ContActionConstructionData ConstructionData;

    ContinuousRobotAction(oppt::ActionSharedPtr& action);

    ContinuousRobotAction(const oppt::ActionSharedPtr& action);

    ContinuousRobotAction(const FloatType* constructionDataVector,
                          size_t& dim);

    virtual ~ContinuousRobotAction() = default;

    std::unique_ptr<abt::Action> copy() const override;

    FloatType distanceTo(abt::Action const& other) const override;
    
    virtual void serialize(std::ostream &os, const std::string prefix="") const override;

    void print(std::ostream& os) const override;

    virtual const abt::ContinuousActionConstructionDataBase& getConstructionData() const override;

    virtual bool equals(Point const& otherPoint) const override;

    virtual std::size_t hash() const override;

    const oppt::ActionSharedPtr getOpptAction() const;

private:
    std::shared_ptr<ConstructionData> storage_;

};

class DiscreteRobotAction: public abt::DiscretizedPoint
{
public:
    /** Constructs a new action from the given ActionType. */
    DiscreteRobotAction(oppt::ActionSharedPtr& action);

    DiscreteRobotAction(const oppt::ActionSharedPtr& action);

    /**DiscreteRobotAction(long code,
                        const VectorFloat& lowerLimits,
                        const VectorFloat& upperLimits,
                        int num_input_steps);*/

    virtual ~DiscreteRobotAction() = default;

    std::unique_ptr<abt::Action> copy() const override;

    FloatType distanceTo(abt::Action const& other) const override;
    
    virtual void serialize(std::ostream &os, const std::string prefix="") const override;

    void print(std::ostream& os) const override;

    long getBinNumber() const override;

    const oppt::ActionSharedPtr getOpptAction() const;

private:
    long bin_number_;

    //std::unique_ptr<robot::RobotAction> action_;
    oppt::ActionSharedPtr action_;

};

}

#endif

