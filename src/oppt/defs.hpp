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
/** @file defs.hpp
 *
 * Some key definitions and macros for C++ programs.
 */
#ifndef DEFS_HPP_
#define DEFS_HPP_

/** A macro for quoting strings. */
#define QUOTE(str) #str
/** A macro that quotes properly, by first expanding other macros, and then quoting that. */
#define EXPAND_AND_QUOTE(str) QUOTE(str)

/** A macro to add static cast members to a class */
#define _STATIC_CAST \
    template<class T> \
    T* as() { \
        return static_cast<T*>(this); \
    } \
    \
    template<class T> \
    T const* as() const { \
        return static_cast<T const*>(this); \
    }

#define _MOVE(ClassName) \
    ClassName(ClassName &&) = default; \
    ClassName &operator=(ClassName &&) = default;

#define _COPY_AND_MOVE(ClassName) \
    ClassName(const ClassName &) = default; \
    ClassName(ClassName &&) = default; \
    ClassName &operator=(const ClassName &) = default; \
    ClassName &operator=(ClassName &&) = default;

/** A macro to delete the copy constructors and assignment operators */
#define _NO_COPY_(classname) \
classname(const classname &) = delete; \
classname &operator=(const classname &) = delete;

/** A macro used to delete all copy- and move-constructors and -assignment operators */
#define _NO_COPY_OR_MOVE(ClassName) \
    ClassName(const ClassName &) = delete; \
    ClassName(ClassName &&) = delete; \
    ClassName &operator=(const ClassName &) = delete; \
    ClassName &operator=(ClassName &&) = delete;

/** A macro used to define default move constructors/assigment operators but and to delete copy constructors/assignment operators */
#define _NO_COPY_BUT_MOVE(ClassName) \
    ClassName(const ClassName &) = delete; \
    ClassName(ClassName &&) = default; \
    ClassName &operator=(const ClassName &) = delete; \
    ClassName &operator=(ClassName &&) = default;
    
/** Decide floating point precision during compile time */
#ifdef USE_DOUBLE_PRECISION
#define FloatType double
#else
#define FloatType float
#endif

// If we're not using C++1y / C++14, we need to define our own std::make_unique
#if __cplusplus <= 201103L
/** We use an inclusion guard to make sure this definition is only included once. */
#ifndef __STD__MAKE__UNIQUE___
#define __STD__MAKE__UNIQUE___
namespace std {
    /** SFINAE struct for single objects. */
    template<class T> struct _Unique_if {
        /** Return type - a single object wrapped in a unique_ptr. */
        typedef unique_ptr<T> _Single_object;
    };

    /** SFINAE struct for arrays with unknown bounds. */
    template<class T> struct _Unique_if<T[]> {
        /** Return type - an array wrapped in a unique_ptr. */
        typedef unique_ptr<T[]> _Unknown_bound;
    };

    /** SFINAE struct for arrays with known bounds. */
    template<class T, size_t N> struct _Unique_if<T[N]> {
        /** No return type, since make_unique is disallowed for arrays with known bounds. */
        typedef void _Known_bound;
    };

    /** The standard single-object make_unique. */
    template<class T, class... Args>
        typename _Unique_if<T>::_Single_object
        make_unique(Args&&... args) {
            return unique_ptr<T>(new T(std::forward<Args>(args)...));
        }

    /** Construction of arrays with unknown bound. */
    template<class T>
        typename _Unique_if<T>::_Unknown_bound
        make_unique(size_t n) {
            typedef typename remove_extent<T>::type U;
            return unique_ptr<T>(new U[n]());
        }

    /** Construction of arrays with a known bound is disallowed. */
    template<class T, class... Args>
        typename _Unique_if<T>::_Known_bound
        make_unique(Args&&...) = delete;
}
#endif
#endif

#endif /* DEFS_HPP_ */
