/** @file parsers.cpp
 *
 * Provides implementations for the parsing of various ABT parameters, allowing for additional
 * customization at runtime instead of compile-time.
 */
#include "parsers.hpp"

#include <iostream>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/Solver.hpp"

#include "solvers/ABT/solver/abstract-problem/heuristics/HeuristicFunction.hpp"

#include "solvers/ABT/solver/belief-estimators/estimators.hpp"

#include "solvers/ABT/solver/search/search_interface.hpp"
#include "solvers/ABT/solver/search/MultipleStrategiesExp3.hpp"
#include "solvers/ABT/solver/search/steppers/ucb_search.hpp"
#include "solvers/ABT/solver/search/steppers/gps_search.hpp"
#include "solvers/ABT/solver/search/steppers/default_rollout.hpp"
#include "solvers/ABT/solver/search/steppers/nn_rollout.hpp"

#include "ModelWithProgramOptions.hpp"

namespace shared {
std::vector<std::string> split_function(std::string text) {
    std::size_t i0 = text.find('(');
    std::size_t i1 = text.rfind(')');
    if (i0 == std::string::npos || i1 == std::string::npos || i1 <= i0) {
        return std::vector<std::string>( { text });
    }

    std::string function = text.substr(0, i0);
    oppt::trim(function);
    std::vector<std::string> argsVector;
    argsVector.push_back(function);

    std::string argsString = text.substr(i0 + 1, i1 - i0 - 1);
    std::string::iterator prevIter = argsString.begin();
    int parenCount = 0;
    for (std::string::iterator charIter = argsString.begin(); charIter != argsString.end();
            charIter++) {
        if (*charIter == '(') {
            parenCount++;
            continue;
        }
        if (*charIter == ')') {
            parenCount--;
            continue;
        }
        if (*charIter == ',' && parenCount == 0) {
            if (prevIter != charIter) {
                std::string s(prevIter, charIter);
                oppt::trim(s);
                argsVector.push_back(s);
            }
            prevIter = charIter + 1;
        }
    }
    if (prevIter != argsString.end()) {
        std::string s(prevIter, argsString.end());
        oppt::trim(s);
        argsVector.push_back(s);
    }
    return argsVector;
}

std::unique_ptr<abt::StepGeneratorFactory> UcbParser::parse(abt::Solver *solver,
        std::vector<std::string> args) {
    FloatType explorationCoefficient;
    std::istringstream(args[1]) >> explorationCoefficient;
    return std::make_unique<abt::UcbStepGeneratorFactory>(solver, explorationCoefficient);
}

std::unique_ptr<abt::StepGeneratorFactory> GpsParser::parse(abt::Solver *solver, std::vector<std::string> args) {

	using abt::choosers::GpsChooserOptions;
	GpsChooserOptions options;

	std::string searchType = "";
	fillOption(args, "searchType", searchType);
	if (searchType == "golden") {
		options.searchType = decltype(options)::GOLDEN;
	} else if (searchType == "compass") {
		options.searchType = decltype(options)::COMPASS;
	} else {
		std::cout << "Warning: unknown gps search type given: " << searchType << std::endl;
	}

	fillOption(args, "dimensions", options.dimensions);
	fillOption(args, "explorationCoefficient", options.explorationCoefficient);
	fillOption(args, "newSearchPointCoefficient", options.newSearchPointCoefficient);
	fillOption(args, "minimumVisitsBeforeChildCreation", options.minimumVisitsBeforeChildCreation);
	fillOption(args, "minimumChildCreationDistance", options.minimumChildCreationDistance);
	fillOption(args, "initialCompassRadiusRatio", options.initialCompassRadiusRatio);

    if (options.newSearchPointCoefficient <= 0) {
    	options.disableGpsSearch = true;
    } else {
    	options.disableGpsSearch = false;
    }

    return std::make_unique<abt::GpsStepGeneratorFactory>(solver, options);
}


std::unique_ptr<abt::StepGeneratorFactory> NnRolloutParser::parse(abt::Solver *solver,
        std::vector<std::string> args) {
    long maxNnComparisons;
    FloatType maxNnDistance;
    std::istringstream(args[1]) >> maxNnComparisons;
    std::istringstream(args[2]) >> maxNnDistance;
    return std::make_unique<abt::NnRolloutFactory>(solver, maxNnComparisons, maxNnDistance);
}
std::unique_ptr<abt::StepGeneratorFactory> DefaultRolloutParser::parse(abt::Solver *solver,
        std::vector<std::string> args) {
    long maxNSteps;
    std::istringstream(args[1]) >> maxNSteps;
    return std::make_unique<abt::DefaultRolloutFactory>(solver, maxNSteps);
}

StagedParser::StagedParser(ParserSet<std::unique_ptr<abt::StepGeneratorFactory>> *allParsers) :
            allParsers_(allParsers) {
}
std::unique_ptr<abt::StepGeneratorFactory> StagedParser::parse(abt::Solver *solver,
        std::vector<std::string> args) {
    std::vector<std::unique_ptr<abt::StepGeneratorFactory>> factories;
    for (auto it = args.begin() + 1; it != args.end(); it++) {
        factories.push_back(allParsers_->parse(solver, *it));
    }
    return std::make_unique<abt::StagedStepGeneratorFactory>(std::move(factories));
}

DefaultHeuristicParser::DefaultHeuristicParser(ModelWithProgramOptions *model) :
        heuristic_() {
    heuristic_ = [model] (abt::HistoryEntry const *entry,
            abt::State const *state, abt::HistoricalData const *data) {
        return model->getDefaultHeuristicValue(entry, state, data);
    };
}
abt::HeuristicFunction DefaultHeuristicParser::parse(
        abt::Solver */*solver*/, std::vector<std::string> /*args*/) {
    return heuristic_;
}
abt::HeuristicFunction ZeroHeuristicParser::parse(
        abt::Solver */*solver*/, std::vector<std::string> /*args*/) {
    return [] (abt::HistoryEntry const *, abt::State const *,
            abt::HistoricalData const *) {
        return 0;
    };
}

BasicSearchParser::BasicSearchParser(
        ParserSet<std::unique_ptr<abt::StepGeneratorFactory>> *generatorParsers,
        ParserSet<abt::HeuristicFunction> *heuristicParsers, std::string heuristicString) :
            generatorParsers_(generatorParsers),
            heuristicParsers_(heuristicParsers),
            heuristicString_(heuristicString) {
}

std::unique_ptr<abt::SearchStrategy> BasicSearchParser::parse(abt::Solver *solver,
        std::vector<std::string> args) {
    std::unique_ptr<abt::StepGeneratorFactory> factory = nullptr;
    std::string heuristicString = heuristicString_;
    if (args[0] != "basic") {
        factory = generatorParsers_->parse(solver, args);
    } else {
        factory = generatorParsers_->parse(solver, args[1]);
        if (args.size() > 2) {
            heuristicString = args[2];
        }
    }

    return std::make_unique<abt::BasicSearchStrategy>(solver, std::move(factory),
            heuristicParsers_->parse(solver, heuristicString));
}

Exp3Parser::Exp3Parser(ParserSet<std::unique_ptr<abt::SearchStrategy>> *allParsers) :
        allParsers_(allParsers) {
}
std::unique_ptr<abt::SearchStrategy> Exp3Parser::parse(abt::Solver *solver,
        std::vector<std::string> args) {
    std::vector<std::unique_ptr<abt::SearchStrategy>> strategies;
    std::vector<std::string>::iterator it = args.begin() + 1;
    FloatType strategyExplorationCoefficient;
    std::istringstream(*it) >> strategyExplorationCoefficient;
    it++;
    for (; it != args.end(); it++) {
        strategies.push_back(allParsers_->parse(solver, *it));
    }
    return std::make_unique<abt::MultipleStrategiesExp3>(solver, strategyExplorationCoefficient,
            std::move(strategies));
}

std::unique_ptr<abt::EstimationStrategy> AverageEstimateParser::parse(abt::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<abt::EstimationFunction>(
            std::function<FloatType(abt::BeliefNode const *)>(abt::estimators::average));
}
std::unique_ptr<abt::EstimationStrategy> MaxEstimateParser::parse(abt::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<abt::EstimationFunction>(abt::estimators::max);
}
std::unique_ptr<abt::EstimationStrategy> RobustEstimateParser::parse(abt::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<abt::EstimationFunction>(abt::estimators::robust);
}


std::unique_ptr<abt::SelectRecommendedActionStrategy> MaxRecommendedActionStrategyParser::parse(abt::Solver * /*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<abt::MaxRecommendedActionStrategy>();
}

std::unique_ptr<abt::SelectRecommendedActionStrategy> GpsMaxRecommendedActionStrategyParser::parse(abt::Solver * /*solver*/,
        std::vector<std::string> args) {

	using abt::choosers::GpsMaxRecommendationOptions;
	GpsMaxRecommendationOptions options;

	std::string searchType = "";
	fillOption(args, "searchType", searchType);
	if (searchType == "golden") {
		options.searchType = decltype(options)::GOLDEN;
	} else if (searchType == "compass") {
		options.searchType = decltype(options)::COMPASS;
	} else {
		std::cout << "Warning: unknown gps search type given: " << searchType << std::endl;
	}


	fillOption(args, "dimensions", options.dimensions);

	std::string recommendationMode = "";
	fillOption(args, "recommendationMode", recommendationMode);
	if (recommendationMode == "mean") {
		options.recommendationMode = decltype(options)::MEAN;
	} else if (recommendationMode == "robust") {
		options.recommendationMode = decltype(options)::ROBUST;
	} else {
		std::cout << "Warning: unknown recommendation mode given: " << recommendationMode << std::endl;
	}

    return std::make_unique<abt::GpsMaxRecommendedActionStrategy>(options);
}


} /* namespace shared */
