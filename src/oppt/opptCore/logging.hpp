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
#ifndef __OPPT_LOGGING_HPP__
#define __OPPT_LOGGING_HPP__
#include <iostream>
#include <stdexcept>

namespace oppt
{

extern bool verbose_;

inline void setVerbose(const bool& verbose)
{
    verbose_ = verbose;
}

inline std::string splitP(const std::string& s, char delim)
{
    std::stringstream ss(s);
    std::string item;
    std::string elem = s;
    while (getline(ss, item, delim)) {
        elem = item;
    }

    return elem;
}

inline void OPPTLOG(const std::string& message)
{
    if (verbose_)
        std::clog << "\033[32m " << message << "\33[39m" << std::endl;
}

inline void OPPTWARN(const std::string& message)
{
    if (verbose_)
        std::clog << "\033[33m WARNING: " << message << "\33[39m" << std::endl;
}

inline void OPPTERROR(std::string message)
{
    std::cerr << "\033[31m ERROR: " << message << "\33[39m" << std::endl;
    abort();
    //throw std::runtime_error(message);
}

inline void OPPTPRINT(std::string message)
{
    if (verbose_)
        std::cout << message << "\n";
}

#define LOGGING(message) OPPTLOG(splitP(std::string(__FILE__), '/') + " " + std::to_string(__LINE__) + ": " + message);
#define WARNING(message) OPPTWARN(splitP(std::string(__FILE__), '/') + " " + std::to_string(__LINE__) + ": " + message);
#define ERROR(message) OPPTERROR(splitP(std::string(__FILE__), '/') + " " + std::to_string(__LINE__) + ": " + message);
#define PRINT(message) OPPTPRINT(message);

}

#endif

