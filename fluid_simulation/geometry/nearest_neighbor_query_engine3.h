#pragma once
#ifndef NEAREST_NEIGHBOR_QUERY_ENGINE3
#define NEAREST_NEIGHBOR_QUERY_ENGINE3

#include "../common_utils.h"
#include <functional>

//! Nearest neighbor query result.
template <typename T>
struct NearestNeighborQueryResult3 {
    const T* item = nullptr;
    double distance = std::numeric_limits<double>::max();
};

//! Nearest neighbor distance measure function.
template <typename T>
using NearestNeighborDistanceFunc3 =
std::function<double(const T&, const Vector3D&)>;

//! Abstract base class for 3-D nearest neigbor query engine.
template <typename T>
class NearestNeighborQueryEngine3 {
public:
    //! Returns the nearest neighbor for given point and distance measure
    //! function.
    virtual NearestNeighborQueryResult3<T> nearest(
        const Vector3D& pt,
        const NearestNeighborDistanceFunc3<T>& distanceFunc) const = 0;
};

#endif 