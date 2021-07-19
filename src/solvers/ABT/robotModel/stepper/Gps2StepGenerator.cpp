#include "Gps2StepGenerator.hpp"

namespace oppt
{
Gps2StepGenerator::Gps2StepGenerator(abt::SearchStatus& status, abt::Solver* solver, abt::choosers::GpsChooserOptions options):
    abt::GpsStepGenerator(status, solver, options)
{

}

Gps2StepGeneratorFactory::Gps2StepGeneratorFactory(abt::Solver* solver, abt::choosers::GpsChooserOptions options):
    abt::GpsStepGeneratorFactory(solver, options)
{

}

std::unique_ptr< abt::StepGeneratorFactory > Gps2Parser::parse(abt::Solver* solver, std::vector< std::string > args)
{
    using abt::choosers::GpsChooserOptions;
    abt::choosers::GpsChooserOptions options;

    std::string searchType = "";
    shared::fillOption(args, "searchType", searchType);
    if (searchType == "golden") {
        options.searchType = decltype(options)::GOLDEN;
    } else if (searchType == "compass") {
        options.searchType = decltype(options)::COMPASS;
    } else {
        std::cout << "Warning: unknown gps search type given: " << searchType << std::endl;
    }
    
    options.dimensions = dimensions_;
    shared::fillOption(args, "explorationCoefficient", options.explorationCoefficient);
    shared::fillOption(args, "newSearchPointCoefficient", options.newSearchPointCoefficient);
    shared::fillOption(args, "minimumVisitsBeforeChildCreation", options.minimumVisitsBeforeChildCreation);
    shared::fillOption(args, "minimumChildCreationDistance", options.minimumChildCreationDistance);
    shared::fillOption(args, "initialCompassRadiusRatio", options.initialCompassRadiusRatio);

    if (options.newSearchPointCoefficient <= 0) {
        options.disableGpsSearch = true;
    } else {
        options.disableGpsSearch = false;
    }

    return std::make_unique<Gps2StepGeneratorFactory>(solver, options);

}

void Gps2Parser::setDimensions(const unsigned int& dimensions)
{
    dimensions_ = dimensions;
}

std::unique_ptr< abt::SelectRecommendedActionStrategy > Gps2MaxRecommendedActionStrategyParser::parse(abt::Solver* solver, std::vector< std::string > args)
{
    using abt::choosers::GpsMaxRecommendationOptions;
    GpsMaxRecommendationOptions options;

    std::string searchType = "";
    shared::fillOption(args, "searchType", searchType);
    if (searchType == "golden") {
        options.searchType = decltype(options)::GOLDEN;
    } else if (searchType == "compass") {
        options.searchType = decltype(options)::COMPASS;
    } else {
        std::cout << "Warning: unknown gps search type given: " << searchType << std::endl;
    }    
    options.dimensions = dimensions_;

    std::string recommendationMode = "";
    shared::fillOption(args, "recommendationMode", recommendationMode);
    if (recommendationMode == "mean") {
        options.recommendationMode = decltype(options)::MEAN;
    } else if (recommendationMode == "robust") {
        options.recommendationMode = decltype(options)::ROBUST;
    } else {
        std::cout << "Warning: unknown recommendation mode given: " << recommendationMode << std::endl;
    }

    return std::make_unique<abt::GpsMaxRecommendedActionStrategy>(options);
}

void Gps2MaxRecommendedActionStrategyParser::setDimensions(const unsigned int& dimensions)
{
    dimensions_ = dimensions;
}


}
