#pragma once
#ifndef IMPLICIT_SURFACE_SET3
#define IMPLICIT_SURFACE_SET3

#include "bvh3.h"
#include "implicit_surface3.h"
#include <vector>

//! \brief 3-D implicit surface set.
//!
//! This class represents 3-D implicit surface set which extends
//! ImplicitSurface3 by overriding implicit surface-related quries. This is
//! class can hold a collection of other implicit surface instances.
//!
class ImplicitSurfaceSet3 final : public ImplicitSurface3 {
public:

    //! Constructs an empty implicit surface set.
    ImplicitSurfaceSet3();

    //! Constructs an implicit surface set using list of other surfaces.
    ImplicitSurfaceSet3(const std::vector<ImplicitSurface3Ptr>& surfaces,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Constructs an implicit surface set using list of other surfaces.
    ImplicitSurfaceSet3(const std::vector<Surface3Ptr>& surfaces,
        const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Copy constructor.
    ImplicitSurfaceSet3(const ImplicitSurfaceSet3& other);

    //this is an extra function
    void set_explicit_surfaces(const std::vector<Surface3Ptr>& surfaces);

    //! Updates internal spatial query engine.
    void updateQueryEngine() override;

    //! Returns true if bounding box can be defined.
    bool isBounded() const override;

    //! Returns true if the surface is a valid geometry.
    bool isValidGeometry() const override;

    //! Returns the number of implicit surfaces.
    size_t numberOfSurfaces() const;

    //! Returns the i-th implicit surface.
    const ImplicitSurface3Ptr& surfaceAt(size_t i) const;

    //! Adds an explicit surface instance.
    void addExplicitSurface(const Surface3Ptr& surface);

    //! Adds an implicit surface instance.
    void addSurface(const ImplicitSurface3Ptr& surface);

private:
    std::vector<ImplicitSurface3Ptr> _surfaces;
    std::vector<ImplicitSurface3Ptr> _unboundedSurfaces;
    mutable Bvh3<ImplicitSurface3Ptr> _bvh;
    mutable bool _bvhInvalidated = true;

    // Surface3 implementations

    Vector3D closestPointLocal(const Vector3D& otherPoint) const override;

    BoundingBox3D boundingBoxLocal() const override;

    double closestDistanceLocal(const Vector3D& otherPoint) const override;

    bool intersectsLocal(const Ray3D& ray) const override;

    Vector3D closestNormalLocal(const Vector3D& otherPoint) const override;

    SurfaceRayIntersection3 closestIntersectionLocal(
        const Ray3D& ray) const override;

    bool isInsideLocal(const Vector3D& otherPoint) const override;

    // ImplicitSurface3 implementations

    double signedDistanceLocal(const Vector3D& otherPoint) const override;

    void invalidateBvh();

    void buildBvh() const;
};

//! Shared pointer type for the ImplicitSurfaceSet3.
typedef std::shared_ptr<ImplicitSurfaceSet3> ImplicitSurfaceSet3Ptr;


#endif