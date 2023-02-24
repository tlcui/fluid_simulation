#pragma once
#ifndef SURFACE3_H_
#define SURFACE3_H_

//the following code is copied from 
#include "bounding_box3.h"
#include "ray3.h"
#include "transform3.h"

struct SurfaceRayIntersection3 {
    bool isIntersecting = false;
    double distance = std::numeric_limits<double>::max();
    Vector3D point;
    Vector3D normal;
};

//! Abstract base class for 3-D surface.
class Surface3 {
public:
    //! Local-to-world transform.
    Transform3 transform;

    //! Flips normal when calling Surface3::closestNormal(...).
    //! this is used for fluid containers
    bool isNormalFlipped = false;

    //! Constructs a surface with normal direction.
    Surface3(const Transform3& transform = Transform3(),
        bool isNormalFlipped = false);

    //! Copy constructor.
    Surface3(const Surface3& other);

    //! Default destructor.
    virtual ~Surface3();

    //! Returns the closest point from the given point \p otherPoint to the
    //! surface.
    Vector3D closestPoint(const Vector3D& otherPoint) const;

    //! Returns the bounding box of this surface object.
    BoundingBox3D boundingBox() const;

    //! Returns true if the given \p ray intersects with this surface object.
    bool intersects(const Ray3D& ray) const;

    //! Returns the closest distance from the given point \p otherPoint to the
    //! point on the surface.
    double closestDistance(const Vector3D& otherPoint) const;

    //! Returns the closest intersection point for given \p ray.
    SurfaceRayIntersection3 closestIntersection(const Ray3D& ray) const;

    //! Returns the normal to the closest point on the surface from the given
    //! point \p otherPoint.
    Vector3D closestNormal(const Vector3D& otherPoint) const;

    //! Updates internal spatial query engine.
    virtual void updateQueryEngine();

    //! Returns true if bounding box can be defined.
    virtual bool isBounded() const;

    //! Returns true if the surface is a valid geometry.
    virtual bool isValidGeometry() const;

    //! Returns true if \p otherPoint is inside the volume defined by the
    //! surface.
    bool isInside(const Vector3D& otherPoint) const;

protected:
    //! Returns the closest point from the given point \p otherPoint to the
    //! surface in local frame.
    virtual Vector3D closestPointLocal(const Vector3D& otherPoint) const = 0;

    //! Returns the bounding box of this surface object in local frame.
    virtual BoundingBox3D boundingBoxLocal() const = 0;

    //! Returns the closest intersection point for given \p ray in local frame.
    virtual SurfaceRayIntersection3 closestIntersectionLocal(
        const Ray3D& ray) const = 0;

    //! Returns the normal to the closest point on the surface from the given
    //! point \p otherPoint in local frame.
    virtual Vector3D closestNormalLocal(const Vector3D& otherPoint) const = 0;

    //! Returns true if the given \p ray intersects with this surface object
    //! in local frame.
    virtual bool intersectsLocal(const Ray3D& ray) const;

    //! Returns the closest distance from the given point \p otherPoint to the
    //! point on the surface in local frame.
    virtual double closestDistanceLocal(const Vector3D& otherPoint) const;

    //! Returns true if \p otherPoint is inside by given \p depth the volume
    //! defined by the surface in local frame.
    virtual bool isInsideLocal(const Vector3D& otherPoint) const;
};

//! Shared pointer for the Surface3 type.
typedef std::shared_ptr<Surface3> Surface3Ptr;

#endif 