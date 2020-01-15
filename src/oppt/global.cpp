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
/** @file global.cpp
 *
 * Implementation for show_message() - a great place for breakpoints!
 */
#include "global.hpp"

#include <iostream>
#include <unistd.h>

//#include "oppt/opptCore/logging.hpp"

namespace oppt
{

namespace UID
{
/** A unique global ID that is incremented when using getUniqueId() */
long globalId = 0;
}

/** Verbose output */
bool verbose_ = false;

std::string get_current_directory()
{
    char* buffer = getcwd(nullptr, 0);
    if (buffer == nullptr) {
        debug::show_message("ERROR: Failed to get current path.");
        std::exit(4);
    }
    std::string dir(buffer);
    free(buffer);
    return dir;
}

void change_directory(std::string& dir)
{
    if (chdir(dir.c_str())) {
        std::ostringstream oss;
        oss << "ERROR: Failed to change path to " << dir;
        debug::show_message(oss.str());
        std::exit(3);
    }
}
} /* namespace oppt */


namespace debug
{
void show_message(std::string message, bool print, bool bp_branch)
{
    if (print) {
        std::cerr << message << std::endl;
    }
    if (bp_branch) {
        std::cerr << "";
    }
}

} /* namespace oppt */
