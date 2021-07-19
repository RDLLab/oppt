#ifndef __GPS2_STEP_GENERATOR_HPP__
#define __GPS2_STEP_GENERATOR_HPP__
#include "oppt/opptCore/core.hpp"
#include "solvers/ABT/robotModel/parsers.hpp"
#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "solvers/ABT/solver/search/steppers/gps_search.hpp"
#include "solvers/ABT/robotModel/ModelWithProgramOptions.hpp"

namespace oppt
{
class Gps2StepGenerator : public abt::GpsStepGenerator
{
public:
    Gps2StepGenerator(abt::SearchStatus& status, abt::Solver* solver, abt::choosers::GpsChooserOptions options);

};

class Gps2StepGeneratorFactory: public abt::GpsStepGeneratorFactory
{
public:
    Gps2StepGeneratorFactory(abt::Solver* solver, abt::choosers::GpsChooserOptions options);
};

class Gps2Parser: public shared::Parser<std::unique_ptr<abt::StepGeneratorFactory>>
{
public:
    Gps2Parser() = default;

    virtual ~Gps2Parser() = default;

    virtual std::unique_ptr<abt::StepGeneratorFactory> parse(abt::Solver* solver,
            std::vector<std::string> args) override;

    void setDimensions(const unsigned int& dimensions);

private:
    unsigned int dimensions_ = 0;

};

class Gps2MaxRecommendedActionStrategyParser: public shared::Parser<std::unique_ptr<abt::SelectRecommendedActionStrategy>>
{
public:
    Gps2MaxRecommendedActionStrategyParser() = default;
    virtual ~Gps2MaxRecommendedActionStrategyParser() = default;
    virtual std::unique_ptr<abt::SelectRecommendedActionStrategy> parse(abt::Solver* solver,
            std::vector<std::string> args) override;

    void setDimensions(const unsigned int& dimensions);

private:
    unsigned int dimensions_ = 0;
};

}


#endif
