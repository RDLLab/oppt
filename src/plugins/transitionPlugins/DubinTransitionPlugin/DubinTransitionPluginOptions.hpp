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
#ifndef _DUBIN_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _DUBIN_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class DubinTransitionPluginOptions: public PluginOptions
{
public:
    DubinTransitionPluginOptions() = default;

    virtual ~DubinTransitionPluginOptions() = default;

    /** @brief The process error the actions are affected to */
    FloatType processError = 0.0;
    
    /** @brief The control duration */
    FloatType controlDuration = 0.3;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addGazeboTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addGazeboTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "processError",
                                     &DubinTransitionPluginOptions::processError);
	
	parser->addOption<FloatType>("transitionPluginOptions",
                                     "processError",
                                     &DubinTransitionPluginOptions::processError);
    }

};
}

#endif