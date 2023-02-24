#pragma once
#ifndef CYLINDER3_H_
#define CYLINDER3_H_

#include "surface3.h"

//! \brief 3-D cylinder geometry.
//!
//! This class represents 3-D cylinder geometry which extends Surface3 by
//! overriding surface-related queries. The cylinder is aligned with the y-axis.
//!
class Cylinder3 final : public Surface3 {
public:
    //class Builder;

    //! Center of the cylinder.
    Vector3D center = Vector3D::Zero();

    //! Radius of the cylinder.
    double radius = 1.0;

    //! Height of the cylinder.
    double height = 1.0;

    //! Constructs a cylinder with
    Cylinder3(
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Constructs a cylinder with \p center, \p radius, and \p height.
    Cylinder3(
        const Vector3D& center,
        double radius,
        double height,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Copy constructor.
    Cylinder3(const Cylinder3& other);

    //! Returns builder fox Cylinder3.
    //static Builder builder();

protected:
    // Surface3 implementations

    Vector3D closestPointLocal(const Vector3D& otherPoint) const override;

    double closestDistanceLocal(const Vector3D& otherPoint) const override;

    bool intersectsLocal(const Ray3D& ray) const override;

    BoundingBox3D boundingBoxLocal() const override;

    Vector3D closestNormalLocal(const Vector3D& otherPoint) const override;

    SurfaceRayIntersection3 closestIntersectionLocal(
        const Ray3D& ray) const override;
};

//! Shared pointer type for the Cylinder3.
typedef std::shared_ptr<Cylinder3> Cylinder3Ptr;

#endif