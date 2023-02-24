#pragma once
#ifndef BOX3_H_
#define BOX3_H_

#include "bounding_box3.h"
#include "surface3.h"

//! \brief 3-D box geometry.
//!
//! This class represents 3-D box geometry which extends Surface3 by overriding
//! surface-related queries. This box implementation is an axis-aligned box
//! that wraps lower-level primitive type, BoundingBox3D.
//!
class Box3 final : public Surface3 {
public:
    //class Builder;

    //! Bounding box of this box.
    BoundingBox3D bound
        = BoundingBox3D(Vector3D::Zero(), Vector3D(1.0, 1.0, 1.0));

    //! Constructs (0, 0, 0) x (1, 1, 1) box.
    Box3(
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Constructs a box with given \p lowerCorner and \p upperCorner.
    Box3(
        const Vector3D& lowerCorner,
        const Vector3D& upperCorner,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Constructs a box with BoundingBox3D instance.
    explicit Box3(
        const BoundingBox3D& boundingBox,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Copy constructor.
    Box3(const Box3& other);

    //! Returns builder fox Box3.
    //static Builder builder();

protected:
    // Surface3 implementations

    Vector3D closestPointLocal(const Vector3D& otherPoint) const override;

    bool intersectsLocal(const Ray3D& ray) const override;

    BoundingBox3D boundingBoxLocal() const override;

    Vector3D closestNormalLocal(const Vector3D& otherPoint) const override;

    SurfaceRayIntersection3 closestIntersectionLocal(
        const Ray3D& ray) const override;
};

//! Shared pointer type for the Box3.
typedef std::shared_ptr<Box3> Box3Ptr;
#endif