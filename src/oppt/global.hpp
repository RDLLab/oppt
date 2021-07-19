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
/** @file global.hpp
 *
 * Contains some key definitions used throughout the OPPT code.
 */
#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_

#include <unistd.h>
#include <cctype>
#include <ctime>

#include <algorithm>
#include <functional>
#include <locale>
#include <memory>                       // for unique_ptr
#include <random>                       // for default_random_engine
#include <sstream>
#include <utility>                      // for forward

#include "oppt/defs.hpp"

//#include "solver/abstract-problem/Point.hpp"

/** The RNG engine to use throughout the code. */
typedef std::default_random_engine RandomGenerator;



/** A global utility namespace; holds some global functions that are used throughout the OPPT code,
 * as well as some basic data structures.
 */
namespace oppt {
/** A function to return the current working directory. */
std::string get_current_directory();
/** A function to change the current working directory. */
void change_directory(std::string &dir);

/** The global seed used for the RandomEngine */
static int globalSeed = 0;

/** Returns the time (in ms) since the program started running. */
inline FloatType clock_ms() {
    return std::clock() * 1000.0 / CLOCKS_PER_SEC;
}

/** A template method to combine hash values - from boost::hash_combine */
template<class T>
inline void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

/** Trim the string from the left. */
static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

/** Trim the string from the right. */
static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

/** Trim the string from the left and right. */
static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}

/** A simple method to print a value with a fixed width. */
template <typename T>
void print_with_width(T value , std::ostream &os, int width,
        std::ios_base::fmtflags flags = std::ios_base::fixed) {
    std::streamsize oldWidth = os.width(width);
    std::ios_base::fmtflags oldFlags = os.flags(flags);
    os << value;
    os.flags(oldFlags);
    os.width(oldWidth);
}

/** A method to print a floating point value, with various options. */
inline void print_double(FloatType value, std::ostream &os, int width = 10,
        int precision = 7,
        std::ios_base::fmtflags flags = std::ios_base::fixed,
        std::ostream::char_type fillCh = ' ') {
    std::streamsize oldPrecision = os.precision(precision);
    std::streamsize oldWidth = os.width(width);
    std::ostream::char_type oldFillCh = os.fill(fillCh);
    std::ios_base::fmtflags oldflags = os.flags(flags);
    os << value;
    os.flags(oldflags);
    os.fill(oldFillCh);
    os.width(oldWidth);
    os.precision(oldPrecision);
}
} /* namespace oppt */


/** A namespace for global functions that serve useful debugging purposes. */
namespace debug {
    /** Global debug method to display an error / warning message.
     * This method is a great place for setting breakpoints!
     */
    void show_message(std::string message, bool print = true,
            bool bp_branch = true);

    /** A generic method for converting values to strings. This is useful in GDB, which
     * doesn't deal very nicely with using streams directly.
     */
    template <typename T>
    std::string to_string(T t) {
        std::ostringstream sstr;
        sstr << t;
        return sstr.str();
    }
} /* namespace debug */


#endif /* GLOBAL_HPP_ */
