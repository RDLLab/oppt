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
#ifndef __OPPT_UTILS_HPP__
#define __OPPT_UTILS_HPP__
#include "includes.hpp"
#include "logging.hpp"
#include "RobotState.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace oppt
{

inline std::vector<std::string> split(const std::string& s, char delim, std::vector<std::string>& elems)
{
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline void split(const std::string& s, const std::string& delimiter, std::vector<std::string>& elems)
{
    size_t pos = 0;
    std::string token;
    std::string sCopy = s;
    while ((pos = sCopy.find(delimiter)) != std::string::npos) {
        token = sCopy.substr(0, pos);
        elems.push_back(token);
        sCopy.erase(0, pos + delimiter.length());
    }

    elems.push_back(sCopy);
}

/**
 * @brief Prints a std::vector
 * @tparam T The vector element type
 * @param vec The vector to print
 * @param str Prefix that is printed before the vector
 */
template<class T>
inline void printVector(std::vector<T> vec, std::string str = "")
{
    if (str != "")
        cout << str << ": ";
    for (auto & k : vec) {
        cout << k << ", ";
    }

    cout << endl;
}

/**
 * @brief Converts a VectorFloat to Vectordf
 */
inline Vectordf toEigenVec(VectorFloat& vec)
{
    Eigen::Map<Vectordf> e_vec(vec.data(), vec.size());
    return e_vec;

}

/**
 * @brief Converts a const VectorFloat to Vectordf
 */
inline Vectordf toEigenVec(const VectorFloat& vec)
{
    VectorFloat vec2(vec);
    Eigen::Map<Vectordf> eVec(vec2.data(), vec2.size());
    return eVec;
}

/**
 * @brief Converts a oppt::GZVector3 to oppt::Vector3f
 */
inline Vector3f toEigenVec(const GZVector3 &vec) {
#ifdef GZ_GT_7
    Vector3f eVec(vec.X(), vec.Y(), vec.Z());
#else
    Vector3f eVec(vec.x, vec.y, vec.z);
#endif
    return eVec;
}

/**
 * @brief Converts a oppt::Vector3f to oppt::GZVector3
 */
inline GZVector3 toGZVec3(const Vector3f &vec) {
    GZVector3 gzVec(vec[0], vec[1], vec[2]);
    return gzVec;
}

/**
 * @brief Converts a oppt::VectorFloat to oppt::GZVector3
 */
inline GZVector3 toGZVec3(const VectorFloat &vec) {
    if (vec.size() != 3)
        ERROR("Request to convert vector of size " + std::to_string(vec.size()) + " to GZVector3");
    return GZVector3(vec[0], vec[1], vec[2]);
}

/**
 * @brief Converts a Vectordf to std::vector<T>
 */
template<typename T>
inline std::vector<T> toStdVec(const Vectordf& vec) {
    std::vector<T> res;
    res.resize(vec.size());
    Vectordf::Map(&res[0], vec.size()) = vec;
    return res;
}

template<typename T>
inline std::vector<T> addVectors(const std::vector<T> &vec1, const std::vector<T> &vec2) {
    std::vector<T> res(vec1.size());
    [](const std::vector<T> & vec1, const std::vector<T> & vec2, std::vector<T> & res) {
        for (size_t i = 0; i != vec1.size(); ++i) {
            res[i] = vec1[i] + vec2[i];
        }
    }(vec1, vec2, res);

    return res;
}

template<typename T>
inline std::vector<T> subtractVectors(const std::vector<T> &vec1, const std::vector<T> &vec2) {
    std::vector<T> res(vec1.size());
    [](const std::vector<T> & vec1, const std::vector<T> & vec2, std::vector<T> & res) {
        for (size_t i = 0; i != vec1.size(); ++i) {
            res[i] = vec1[i] - vec2[i];
        }
    }(vec1, vec2, res);

    return res;
}

template<typename T>
inline std::vector<T> normalizeVector(const std::vector<T> &vec) {
    FloatType sum = std::accumulate(vec.begin(),
                                    vec.end(),
                                    0.0,
    [](const T & a, const T & b) {
        return a + b * b;
    });
    sum = sqrt(sum);
    std::vector<T> res(vec.size());
    for (size_t i = 0; i != vec.size(); ++i) {
        res[i] = vec[i] / sum;
    }

    return res;
}

template <typename T>
inline std::vector<T> scaleVector(const std::vector<T> &vec, const T &scalar) {
    std::vector<T> res(vec.size());
    for (size_t i = 0; i != vec.size(); ++i) {
        res[i] = scalar * vec[i];
    }

    return res;
}

/**
 * @brief Check if a std::vector of type T contains a specific element
 * @tparam T The vector element type
 * @param vector The vector that is checked from
 * @param elem The element that is checked for
 */
template<typename T>
inline bool contains(const std::vector<T>& vector, const T& elem)
{
    for (auto & entry : vector) {
        if (entry == elem)
            return true;
    }

    return false;
}

}

#endif
