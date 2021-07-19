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
#ifndef __PLUGIN_OPTIONS_HPP__
#define __PLUGIN_OPTIONS_HPP__
#include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"

namespace oppt
{
struct PluginOptions: public oppt::ProblemEnvironmentOptions
{
public:
    PluginOptions() = default;
    virtual ~PluginOptions() = default;

    static std::unique_ptr<options::OptionParser> makeParser() {
	std::unique_ptr<options::OptionParser> parser =
            ProblemEnvironmentOptions::makeParser(false);
        return std::move(parser);
    }

};

}

#endif
