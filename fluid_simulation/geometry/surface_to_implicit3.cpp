#include "surface_to_implicit3.h"


SurfaceToImplicit3::SurfaceToImplicit3(const Surface3Ptr& surface,
    const Transform3& transform,
    bool isNormalFlipped)
    : ImplicitSurface3(transform, isNormalFlipped), _surface(surface) {
    /*if (std::dynamic_pointer_cast<TriangleMesh3>(surface) != nullptr) {
        JET_WARN << "Using TriangleMesh3 with SurfaceToImplicit3 is accurate "
            "but slow. ImplicitTriangleMesh3 can provide faster but "
            "approximated results.";
    }*/
}

SurfaceToImplicit3::SurfaceToImplicit3(const SurfaceToImplicit3& other)
    : ImplicitSurface3(other), _surface(other._surface) {}

bool SurfaceToImplicit3::isBounded() const { return _surface->isBounded(); }

void SurfaceToImplicit3::updateQueryEngine() { _surface->updateQueryEngine(); }

bool SurfaceToImplicit3::isValidGeometry() const {
    return _surface->isValidGeometry();
}

Surface3Ptr SurfaceToImplicit3::surface() const { return _surface; }

Vector3D SurfaceToImplicit3::closestPointLocal(
    const Vector3D& otherPoint) const {
    return _surface->closestPoint(otherPoint);
}

Vector3D SurfaceToImplicit3::closestNormalLocal(
    const Vector3D& otherPoint) const {
    return _surface->closestNormal(otherPoint);
}

double SurfaceToImplicit3::closestDistanceLocal(
    const Vector3D& otherPoint) const {
    return _surface->closestDistance(otherPoint);
}

bool SurfaceToImplicit3::intersectsLocal(const Ray3D& ray) const {
    return _surface->intersects(ray);
}

SurfaceRayIntersection3 SurfaceToImplicit3::closestIntersectionLocal(
    const Ray3D& ray) const {
    return _surface->closestIntersection(ray);
}

BoundingBox3D SurfaceToImplicit3::boundingBoxLocal() const {
    return _surface->boundingBox();
}

double SurfaceToImplicit3::signedDistanceLocal(
    const Vector3D& otherPoint) const {
    Vector3D x = _surface->closestPoint(otherPoint);
    bool inside = _surface->isInside(otherPoint);
    return (inside) ? -(x-otherPoint).norm() : (x-otherPoint).norm();
}

bool SurfaceToImplicit3::isInsideLocal(const Vector3D& otherPoint) const {
    return _surface->isInside(otherPoint);
}