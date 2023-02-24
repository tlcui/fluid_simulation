#pragma once
#ifndef SURFACE_TO_IMPLICIT3
#define SURFACE_TO_IMPLICIT3

#include "implicit_surface3.h"

class SurfaceToImplicit3 final : public ImplicitSurface3 {
public:

    //! Constructs an instance with generic Surface3 instance.
    SurfaceToImplicit3(const Surface3Ptr& surface,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Copy constructor.
    SurfaceToImplicit3(const SurfaceToImplicit3& other);

    //! Updates internal spatial query engine.
    void updateQueryEngine() override;

    //! Returns true if bounding box can be defined.
    bool isBounded() const override;

    //! Returns true if the surface is a valid geometry.
    bool isValidGeometry() const override;

    //! Returns the raw surface instance.
    Surface3Ptr surface() const;

protected:
    Vector3D closestPointLocal(const Vector3D& otherPoint) const override;

    double closestDistanceLocal(const Vector3D& otherPoint) const override;

    bool intersectsLocal(const Ray3D& ray) const override;

    BoundingBox3D boundingBoxLocal() const override;

    Vector3D closestNormalLocal(const Vector3D& otherPoint) const override;

    double signedDistanceLocal(const Vector3D& otherPoint) const override;

    SurfaceRayIntersection3 closestIntersectionLocal(
        const Ray3D& ray) const override;

    bool isInsideLocal(const Vector3D& otherPoint) const override;

private:
    Surface3Ptr _surface;
};

//! Shared pointer for the SurfaceToImplicit3 type.
typedef std::shared_ptr<SurfaceToImplicit3> SurfaceToImplicit3Ptr;


#endif 