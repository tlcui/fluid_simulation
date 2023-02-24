#pragma once
#ifndef PLANE3_H_
#define PLANE3_H_

#include "surface3.h"
//! \brief 3-D plane geometry.
//!
//! This class represents 3-D plane geometry which extends Surface3 by
//! overriding surface-related queries.
//!
class Plane3 final : public Surface3 {
public:
    //class Builder;

    //! Plane normal.
    Vector3D normal = Vector3D(0, 1, 0);

    //! Point that lies on the plane.
    Vector3D point = Vector3D::Zero();

    //! Constructs a plane that crosses (0, 0, 0) with surface normal (0, 1, 0).
    Plane3(
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Constructs a plane that cross \p point with surface normal \p normal.
    Plane3(
        const Vector3D& normal,
        const Vector3D& point,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Constructs a plane with three points on the surface. The normal will be
    //! set using the counter clockwise direction.
    Plane3(
        const Vector3D& point0,
        const Vector3D& point1,
        const Vector3D& point2,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Copy constructor.
    Plane3(const Plane3& other);

    //! Returns true if bounding box can be defined.
    bool isBounded() const override;

    //! Returns builder fox Plane3.
    //static Builder builder();

protected:
    Vector3D closestPointLocal(const Vector3D& otherPoint) const override;

    bool intersectsLocal(const Ray3D& ray) const override;

    BoundingBox3D boundingBoxLocal() const override;

    Vector3D closestNormalLocal(const Vector3D& otherPoint) const override;

    SurfaceRayIntersection3 closestIntersectionLocal(
        const Ray3D& ray) const override;
};

//! Shared pointer for the Plane3 type.
typedef std::shared_ptr<Plane3> Plane3Ptr;

#endif