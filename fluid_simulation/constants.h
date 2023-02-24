#pragma once
#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include<limits>

//! Float-type pi.
constexpr float PI_f = 3.14159265358979323846264338327950288f;

//! Double-type pi.
constexpr double PI_d = 3.14159265358979323846264338327950288;


//! Pi for type T.
template <typename T>
constexpr T PI() {
    return static_cast<T>(PI_d);
}

//! Pi for float.
template <>
constexpr float PI<float>() {
    return PI_f;
}

//! Pi for double.
template <>
constexpr double PI<double>() {
    return PI_d;
}

//! Float-type pi/2.
constexpr float Half_PI_f = 1.57079632679489661923132169163975144f;

//! Double-type pi/2.
constexpr double Half_PI_d = 1.57079632679489661923132169163975144;

//! Pi/2 for type T.
template <typename T>
constexpr T Half_PI() {
    return static_cast<T>(Half_PI_d);
}

//! Pi/2 for float.
template <>
constexpr float Half_PI<float>() {
    return Half_PI_f;
}

//! Pi/2 for double.
template <>
constexpr double Half_PI<double>() {
    return Half_PI_d;
}

//! Float-type 1/pi.
constexpr float Inv_PI_f = static_cast<float>(1.0 / PI_d);

//! Double-type 1/pi.
constexpr double Inv_PI_d = 1.0 / PI_d;

//! 1/pi for type T.
template <typename T>
constexpr T inv_PI() {
    return static_cast<T>(Inv_PI_d);
}

//! 1/pi for float.
template <>
constexpr float inv_PI<float>() {
    return Inv_PI_f;
}

//! 1/pi for double.
template <>
constexpr double inv_PI<double>() {
    return Inv_PI_d;
}
//! epsilon
constexpr float EPSILON_f = std::numeric_limits<float>::epsilon();
constexpr double EPSILON_d = std::numeric_limits<double>::epsilon();

#endif