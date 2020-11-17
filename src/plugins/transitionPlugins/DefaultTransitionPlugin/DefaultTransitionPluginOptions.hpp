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
#ifndef _DEFAULT_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _DEFAULT_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class DefaultTransitionPluginOptions: public PluginOptions
{
public:
    DefaultTransitionPluginOptions() = default;

    virtual ~DefaultTransitionPluginOptions() = default;

    /** @brief The process error the actions are affected to */
    FloatType processError = 0.0;

    /** @brief The time (in seconds) a control has to be applied for */
    FloatType controlDuration = 0.333;

    FloatType softLimitThreshold = 0.0;

    std::string errorDistribution = "GAUSSIAN";

    std::vector<VectorFloat> lowerUpperBoundUniformDistribution;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addDefaultTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addDefaultTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "processError",
                                     &DefaultTransitionPluginOptions::processError);
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "controlDuration",
                                     &DefaultTransitionPluginOptions::controlDuration);
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "softLimitThreshold",
                                     &DefaultTransitionPluginOptions::softLimitThreshold);
        parser->addOption<std::string>("transitionPluginOptions",
                                       "errorDistribution",
                                       &DefaultTransitionPluginOptions::errorDistribution);

        std::vector<VectorFloat> defaultLowerUpperBoundUniformDistribution;
        parser->addOptionWithDefault<std::vector<VectorFloat>>("transitionPluginOptions",
                "lowerUpperBoundUniformDistribution",
                &DefaultTransitionPluginOptions::lowerUpperBoundUniformDistribution, defaultLowerUpperBoundUniformDistribution);
    }

};
}

#endif
