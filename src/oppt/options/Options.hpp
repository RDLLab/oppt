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
/** @file Options.hpp
 *
 * Defines the base Options class. These are the core parameters that ABT requires in order to
 * function correctly.
 */
#ifndef SOLVER_OPTIONS_HPP_
#define SOLVER_OPTIONS_HPP_

#include "oppt/options/option_parser.hpp"

namespace oppt {
/** The base Options class for the ABT solver.
 *
 * This is the data structure into which options are parsed; it is also used by the Model, Solver
 * and Simulator to access the configuration settings, e.g. whether or not verbose output should
 * be printed.
 *
 * The values of numberOfStateVariables, minVal, and maxVal should be set by the model to
 * correctly reflect the specific problem.
 *
 * Additional configuration settings can be added by inheriting from this class.
 */
struct Options : options::BaseOptions {
    Options() = default;
    virtual ~Options() = default;
    
    /** Full path to the config file */
    std::string configPath = "";
    
    /** The path to the log file. */
    std::string logPath = "";   
    
    /** Postfix that will be appended to the log file filename */
    std::string logFilePostfix = "";
      
    /** True iff verbose output is enabled. */
    bool hasVerboseOutput = false;
    
    /* Maximum number of particles to draw */
    unsigned int particlePlotLimit = 30;
};
} /* namespace solver */

#endif /* SOLVER_OPTIONS_HPP_ */
