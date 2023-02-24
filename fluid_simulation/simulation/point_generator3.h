#pragma once
#ifndef POINT_GENERATOR3_H_
#define POINT_GENERATOR3_H_

#include "bounding_box3.h"
#include <memory>
#include <functional>
#include <vector>

class PointGenerator3 {
public:
    PointGenerator3();

    virtual ~PointGenerator3();

    //! Generates points to output array \p points inside given \p boundingBox
    //! with target point \p spacing.
    void generate(
        const BoundingBox3D& boundingBox,
        double spacing,
        std::vector<Vector3D>* points) const;

    //!
    //! \brief Iterates every point within the bounding box with specified
    //! point pattern and invokes the callback function.
    //!
    //! This function iterates every point within the bounding box and invokes
    //! the callback function. The position of the point is specified by the
    //! actual implementation. The suggested spacing between the points are
    //! given by \p spacing. The input parameter of the callback function is
    //! the position of the point and the return value tells whether the
    //! iteration should stop or not.
    //!
    virtual void forEachPoint(
        const BoundingBox3D& boundingBox,
        double spacing,
        const std::function<bool(const Vector3D&)>& callback) const = 0;
};

//! Shared pointer for the PointGenerator3 type.
typedef std::shared_ptr<PointGenerator3> PointGenerator3Ptr;

#endif 
