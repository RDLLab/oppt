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
#ifndef _ROCKSAMPLE_OBSERVATION_PLUGIN_OPTIONS_HPP_
#define _ROCKSAMPLE_OBSERVATION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class RocksampleObservationPluginOptions: public PluginOptions
{
public:
    RocksampleObservationPluginOptions() = default;

    virtual ~RocksampleObservationPluginOptions() = default;

    /** The process error the actions are affected to */
    FloatType halfEfficiencyDistance = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addGazeboObservationPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addGazeboObservationPluginOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("observationPluginOptions",
                                     "halfEfficiencyDistance",
                                     &RocksampleObservationPluginOptions::halfEfficiencyDistance);
    }

};
}

#endif
