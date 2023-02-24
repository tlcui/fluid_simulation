#pragma once
#ifndef RAY_H_
#define RAY_H_

//! \brief      Class for ray.
//!
//! \tparam     T     The value type.
//! \tparam     N     The dimension.
//!
template <typename T, size_t N>
class Ray {
    static_assert(N != 2 && N != 3, "Not implemented.");
    static_assert(
        std::is_floating_point<T>::value,
        "Ray only can be instantiated with floating point types");
};

#endif