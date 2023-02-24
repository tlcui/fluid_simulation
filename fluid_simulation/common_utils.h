#pragma once
#ifndef COMMON_UTILS_H_
#define COMMON_UTILS_H_

#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

typedef long ssize_t;

using Vector3D = Eigen::Vector3d;
using Vector3F = Eigen::Vector3f;

using Vector2D = Eigen::Vector2d;
using Vector2F = Eigen::Vector2f;

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<typename T, unsigned int N>
using Vector = Eigen::Matrix<T, N, 1>;

template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

template<typename T>
using Point3 = Vector3<T>;

using Size3 = Vector3<size_t>;
using Point3I = Vector3<ssize_t>;
using Point3F = Vector3F;
using Point3D = Vector3D;

using Matrix3D = Eigen::Matrix3d;
using Matrix3F = Eigen::Matrix3f;

using QuaternionD = Eigen::Quaterniond;
using QuaternionF = Eigen::Quaternionf;

template<typename T>
inline T clamp(T val, T low, T high)
{
    if (val < low) {
        return low;
    }
    else if (val > high) {
        return high;
    }
    else {
        return val;
    }
}

template <typename T>
Vector<T, 3> min(const Vector<T, 3>& a, const Vector<T, 3>& b) {
    return Vector<T, 3>(std::min(a.x(), b.x()), std::min(a.y(), b.y()),
        std::min(a.z(), b.z()));
}

template <typename T>
Vector<T, 3> max(const Vector<T, 3>& a, const Vector<T, 3>& b) {
    return Vector<T, 3>(std::max(a.x(), b.x()), std::max(a.y(), b.y()),
        std::max(a.z(), b.z()));
}

template <typename T>
Vector<T, 3> clamp(const Vector<T, 3>& v, const Vector<T, 3>& low,
    const Vector<T, 3>& high) {
    return Vector<T, 3>(clamp(v.x(), low.x(), high.x()), clamp(v.y(), low.y(), high.y()),
        clamp(v.z(), low.z(), high.z()));
}

template <typename T>
Vector<T, 3> ceil(const Vector<T, 3>& a) {
    return Vector<T, 3>(std::ceil(a.x()), std::ceil(a.y()), std::ceil(a.z()));
}

template <typename T>
Vector<T, 3> floor(const Vector<T, 3>& a) {
    return Vector<T, 3>(std::floor(a.x()), std::floor(a.y()), std::floor(a.z()));
}

template<typename S, typename T>
inline S lerp(const S& value0, const S& value1, T f) {
    return (1 - f) * value0 + f * value1;
}

template <typename T>
inline T absmin(T x, T y) {
    return (x * x < y* y) ? x : y;
}

template <typename T>
inline T absmax(T x, T y) {
    return (x * x > y * y) ? x : y;
}

#endif