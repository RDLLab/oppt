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
#ifndef __OPPT_MATH_UTILS_HPP__
#define __OPPT_MATH_UTILS_HPP__
#include "typedefs.hpp"
#include "utils.hpp"

#include <iostream>

namespace oppt
{

/**
 * @namespace oppt::math
 * @brief Namespace for math utils
 */
namespace math
{

inline FloatType euclideanDistance(const FloatType* s1, const FloatType* s2, const size_t& dim, const size_t &startIndex=0)
{
    FloatType sum = 0;
    [](const FloatType * s1, const FloatType * s2, FloatType & sum, const size_t & dim, const size_t &startIndex) {
        FloatType diff = 0.0;
        for (size_t i = startIndex; i != dim + startIndex; i++) {
            diff = s1[i] - s2[i];
            sum += diff * diff;
        }

        sum = std::sqrt(sum);
    }(s1, s2, sum, dim, startIndex);

    return sum;
}

/**
 * Euclidean distance between two vectors. Assumes the vectors have the same dimension
 * The vector elements must be of type RealType
 */
template<typename T>
inline T euclideanDistance(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    T sum = 0;
    [&](const std::vector<T>& vec1, const std::vector<T>& vec2, T & sum) {
        for (size_t i = 0; i != vec1.size(); ++i) {
            sum += std::pow(vec2[i] - vec1[i], 2);
        }

        sum = std::sqrt(sum);
    }(vec1, vec2, sum);

    return sum;
}

/**
 * Computes the l1 distance between two vectors
 */
template<typename T>
inline T l1Distance(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    T sum = 0;
    [&](const std::vector<T>& vec1, const std::vector<T>& vec2, T & sum) {
        for (size_t i = 0; i != vec1.size(); ++i) {
            sum += std::fabs(vec1[i] - vec2[i]);
        }
    }(vec1, vec2, sum);

    return sum;
}

/**
 * Computes the l2 norm of a vector
 */
template<typename T>
inline T l2norm(const std::vector<T> &vec) {
    return sqrt(std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0));
}

/**
 * @brief Calculates the mean of a vector of oppt::VectorFloat
 */
inline VectorFloat mean(const std::vector<VectorFloat>& vectors)
{
    VectorFloat mean(vectors[0].size(), 0.0);    
    for (auto & vec : vectors) {
        mean = addVectors(mean, vec);
    }

    return scaleVector(mean, (FloatType)vectors.size());
}

/**
 * @brief Converts a rotation matrix to a quaternion
 */
inline Quaternionf matrixToQuaternion(Matrix3f& matrix)
{
    Quaternionf quat(matrix);
    return quat;
}

/**
 * @brief Calculates the rotation matrix for a rotation about the x-axis
 * @param angle The rotation about the x-axis in rad
 */
inline Matrixdf getRotationMatrixX(const FloatType& angle)
{
    Matrixdf rotationMatrix(4, 4);
    rotationMatrix << 1, 0, 0, 0,
                   0, cos(angle), -sin(angle), 0,
                   0, sin(angle), cos(angle), 0,
                   0, 0, 0, 1;
    return rotationMatrix;

}

/**
 * @brief Calculates the rotation matrix for a rotation about the y-axis
 * @param angle The rotation about the y-axis in rad
 */
inline Matrixdf getRotationMatrixY(const FloatType& angle)
{
    Matrixdf rotationMatrix(4, 4);
    rotationMatrix << cos(angle), 0, sin(angle), 0,
                   0, 1, 0, 0,
                   -sin(angle), 0, cos(angle), 0,
                   0, 0, 0, 1;
    return rotationMatrix;
}

/**
 * @brief Calculates the rotation matrix for a rotation about the z-axis
 * @param angle The rotation about the z-axis in rad
 */
inline Matrixdf getRotationMatrixZ(const FloatType& angle)
{
    Matrixdf rotationMatrix(4, 4);
    rotationMatrix << cos(angle), -sin(angle), 0, 0,
                   sin(angle), cos(angle), 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;
    return rotationMatrix;
}

/**
 * @brief Calculates the translation matrix for translation in xyz-direction
 * @param x Translation in x-direction
 * @param y Translation in y-direction
 * @param z Translation in z-direction
 */
inline Matrixdf getTranslationMatrix(const FloatType& x,
                                     const FloatType& y,
                                     const FloatType& z)
{
    Matrixdf translationMatrix(4, 4);
    translationMatrix << 1, 0, 0, x,
                      0, 1, 0, y,
                      0, 0, 1, z,
                      0, 0, 0, 1;
    return translationMatrix;
}

template<typename T>
inline Quaternionf eulerAnglesToQuaternion(const T& angles)
{
    Matrixdf rotX(4, 4);
    Matrixdf rotY(4, 4);
    Matrixdf rotZ(4, 4);

    Matrixdf rotMatrix = getRotationMatrixX(angles[0]) * getRotationMatrixY(angles[1]) * getRotationMatrixZ(angles[2]);
    Matrix3f rot;
    rot << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2),
        rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2),
        rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2);
    Quaternionf quaternion(rot);
    return quaternion;
}

/**
 * @brief Converts a normalized quaternion to a rotation matrix
 */
inline Matrix3f quaternionToRotationMatrix(const Quaternionf& quat)
{
    return quat.toRotationMatrix();
}

/**
 * @brief Converts a rotation matrix to a quaternion
 */
inline Quaternionf rotationMatrixToQuaternion(const Matrix3f& matrix)
{
    Quaternionf quat(matrix);
    return quat;
}

inline Quaternionf eulerAnglesToQuaternion(const FloatType& roll, const FloatType& pitch, const FloatType& yaw)
{    
    Quaternionf q = AngleAxisf(roll, Vector3f::UnitX()) *
                    AngleAxisf(pitch, Vector3f::UnitY()) *
                    AngleAxisf(yaw, Vector3f::UnitZ());
    return q;
}

inline Vector3f quaternionToEulerAngles(const Quaternionf &q) {
    return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

inline Matrix3f eulerAnglesToRotationMatrix(const FloatType& roll, const FloatType& pitch, const FloatType& yaw)
{
    Matrix3f rotMatrix;
    rotMatrix = AngleAxisf(yaw, Vector3f::UnitZ()) * AngleAxisf(pitch, Vector3f::UnitY()) * AngleAxisf(roll, Vector3f::UnitX());
    //return rotMatrix;
    //rotMatrix = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) * AngleAxisf(yaw, Vector3f::UnitZ());
    return rotMatrix;
}

inline Vector3f rotationMatrixToEulerAngles(const Matrix3f& matrix)
{
    Vector3f eulerAngles = matrix.eulerAngles(2, 1, 0);
    Vector3f swapped;
    swapped << eulerAngles[2], eulerAngles[1], eulerAngles[0];
    return swapped;
}

/**
 * @brief Multiplies two quaternions
 */
inline Quaternionf quatMult(const Quaternionf& q1, const Quaternionf& q2)
{
    Quaternionf resultQ;
    resultQ.setIdentity();
    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());
    return resultQ;
}

/**
 * @brief Converts an angle in degees to radian
 */
inline FloatType degreesToRadian(const FloatType &angle) {
    return angle * M_PI / 180.0;
}

/**
 * @brief Converts an angle in radian to degrees
 */
inline FloatType radianToDegrees(const FloatType &angle) {
    return angle * 180.0 / M_PI;
}

/**
 * @brief Wraps and angle (in radians) to the interval [-pi, pi]
 */
inline FloatType wrapAngle(const FloatType &angle) {
    return atan2(sin(angle), cos(angle));
}

/**
 * @brief Unwrapps an angle (in radians) to the interval [0, 2pi]
 */
inline FloatType unwrapAngle(const FloatType &angle) {
    FloatType piConst = 2.0 * M_PI;
    return fmod(angle + piConst, piConst);
}

/**
 * @brief Calculates the signed difference between angle1 and angle2, i.e. angle1 - angle2
 * @param angle1 The first angle in radians
 * @param angle2 The second angle in radians
 */
inline FloatType distAngles(const FloatType &angle1, const FloatType &angle2) {
    return wrapAngle(angle1 - angle2);    
}

}

}

#endif
