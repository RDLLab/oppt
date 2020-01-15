#ifndef _ABT_OPTIONS_HPP_
#define _ABT_OPTIONS_HPP_
#include "robotModel/SharedOptions.hpp"

namespace oppt
{
struct ABTExtendedOptions: public shared::SharedOptions {
public:
    ABTExtendedOptions() = default;
    virtual ~ABTExtendedOptions() = default;

    /* Maximum amount of time ins seconds to compute the rollout heuristic */
    FloatType heuristicTimeout = 0.1;

    /* The particle filter to use */
    std::string particleFilter = "default";

    /** The minimum number of particles to maintain in the active belief node. */
    unsigned long minParticleCount = 1000;

    /** Allow zero weight particles to be part of the next belief */
    bool allowZeroWeightParticles = false;

    /** The number of new histories to generate on each search step. */
    unsigned long historiesPerStep = 1000;

    /** The maximum depth to search, relative to the current belief node. */
    long maximumDepth = 100;

    /** True if the depth horizon is relative to the starting belief, and false if it is
     * relative to the current belief.
     */
    bool isAbsoluteHorizon = false;

    /* ------------------------- ABT settings --------------------- */
    /** Whether to prune the tree on every simulation step. */
    bool pruneEveryStep = false;
    /** Whether to completely re-build the tree from scratch if changes occur. */
    bool resetOnChanges = false;

    /** Save the policy when done **/
    bool savePolicy = false;

    /** True iff we should load an initial policy before running the simulation. */
    bool loadInitialPolicy = false;

    /** Path to the policy to load */
    std::string policyPath = "";

    /* ---------- ABT settings: advanced customization  ---------- */
    /** The heuristic used for searches. */
    std::string searchHeuristic = "default()";
    /** The search strategy to use. */
    std::string searchStrategy = "";
    /** The recommendation strategy to use. */
    std::string recommendationStrategy = "";
    /** The function to estimate the value of a belief. */
    std::string estimator = "";
    /** The maximum distance between observations to group together; only applicable if
     * approximate observations are in use. */
    FloatType maxObservationDistance = 0.0;
    /** A default lower bound on the value function. */
    FloatType minVal = -std::numeric_limits<FloatType>::infinity();
    /** A default upper bound on the value function. */
    FloatType maxVal = +std::numeric_limits<FloatType>::infinity();

    /** 'discrete' or 'continuous' */
    std::string observationType = "continuous";

    unsigned int numInputStepsObservations = 3;

    /* 'discrete' or 'continuous' */
    std::string actionType = "discrete";

    unsigned int numInputStepsActions = 3;

    std::vector<unsigned int> actionDiscretization = std::vector<unsigned int>();

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            SharedOptions::makeParser(simulating);
        addABTOptions(parser.get());
        addABTExtendedOptions(parser.get());
        return std::move(parser);
    }

    static void addABTOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("ABT", "heuristicTimeout", &ABTExtendedOptions::heuristicTimeout);
        parser->addOption<std::string>("ABT", "particleFilter", &ABTExtendedOptions::particleFilter);
        parser->addOptionWithDefault<bool>("ABT", "pruneEveryStep",
                                           &ABTExtendedOptions::pruneEveryStep, false);
        parser->addOptionWithDefault<bool>("ABT", "resetOnChanges",
                                           &ABTExtendedOptions::resetOnChanges, false);
        parser->addOptionWithDefault<unsigned long>("ABT", "minParticleCount",
                &ABTExtendedOptions::minParticleCount, 1000);
        parser->addOptionWithDefault<bool>("ABT", "allowZeroWeightParticles",
                                           &ABTExtendedOptions::allowZeroWeightParticles, false);
        parser->addOptionWithDefault<unsigned long>("ABT",
                "historiesPerStep",
                &ABTExtendedOptions::historiesPerStep,
                0);
        parser->addOptionWithDefault<long>("ABT", "maximumDepth", &ABTExtendedOptions::maximumDepth, 1000);
        parser->addOption<bool>("ABT", "isAbsoluteHorizon", &ABTExtendedOptions::isAbsoluteHorizon);
        parser->addOption<std::string>("ABT", "estimator", &ABTExtendedOptions::estimator);
        parser->addOption<std::string>("ABT", "observationType", &ABTExtendedOptions::observationType);
        parser->addOption<std::string>("ABT", "actionType", &ABTExtendedOptions::actionType);
        parser->addOptionWithDefault<FloatType>("ABT", "maxObservationDistance",
                                                &ABTExtendedOptions::maxObservationDistance, 0.0);
        parser->addOptionWithDefault<unsigned int>("ABT", "numInputStepsObservations",
                &ABTExtendedOptions::numInputStepsObservations, 3);
        parser->addOptionWithDefault<unsigned int>("ABT", "numInputStepsActions",
                &ABTExtendedOptions::numInputStepsActions, 3);
        std::vector<unsigned int> defaultUIntVec;
        parser->addOptionWithDefault<std::vector<unsigned int>>("ABT",
                "actionDiscretization",
                &ABTExtendedOptions::actionDiscretization, defaultUIntVec);
        parser->addOptionWithDefault<bool>("ABT",
                                           "savePolicy",
                                           &ABTExtendedOptions::savePolicy,
                                           false);
        parser->addOptionWithDefault<bool>("ABT",
                                           "loadInitialPolicy",
                                           &ABTExtendedOptions::loadInitialPolicy,
                                           false);
        parser->addOptionWithDefault<std::string>("ABT",
                "policyPath",
                &ABTExtendedOptions::policyPath,
                "pol.pol");
    }

    static void addABTExtendedOptions(options::OptionParser* parser) {
        parser->addOptionWithDefault<std::string>("ABT",
                "searchStrategy",
                &ABTExtendedOptions::searchStrategy,
                "ucb(2.0)");
        parser->addOptionWithDefault<std::string>("ABT",
                "recommendationStrategy",
                &ABTExtendedOptions::recommendationStrategy,
                "max");
        parser->addOptionWithDefault<FloatType>("ABT",
                                                "minVal",
                                                &ABTExtendedOptions::minVal,
                                                -std::numeric_limits<FloatType>::infinity());
        parser->addOptionWithDefault<FloatType>("ABT",
                                                "maxVal",
                                                &ABTExtendedOptions::maxVal,
                                                std::numeric_limits<FloatType>::infinity());

    }
};
}

#endif
