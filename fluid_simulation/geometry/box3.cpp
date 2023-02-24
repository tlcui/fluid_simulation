#include "box3.h"
#include "plane3.h"

Box3::Box3(const Transform3& transform, bool isNormalFlipped)
    : Surface3(transform, isNormalFlipped) {}

Box3::Box3(const Vector3D& lowerCorner, const Vector3D& upperCorner,
    const Transform3& transform, bool isNormalFlipped)
    : Box3(BoundingBox3D(lowerCorner, upperCorner), transform,
        isNormalFlipped) {}

Box3::Box3(const BoundingBox3D& boundingBox, const Transform3& transform,
    bool isNormalFlipped)
    : Surface3(transform, isNormalFlipped), bound(boundingBox) {}

Box3::Box3(const Box3& other) : Surface3(other), bound(other.bound) {}

Vector3D Box3::closestPointLocal(const Vector3D& otherPoint) const {
    if (bound.contains(otherPoint)) {
        Plane3 planes[6] = { Plane3(Vector3D(1, 0, 0), bound.upperCorner),
                            Plane3(Vector3D(0, 1, 0), bound.upperCorner),
                            Plane3(Vector3D(0, 0, 1), bound.upperCorner),
                            Plane3(Vector3D(-1, 0, 0), bound.lowerCorner),
                            Plane3(Vector3D(0, -1, 0), bound.lowerCorner),
                            Plane3(Vector3D(0, 0, -1), bound.lowerCorner) };

        Vector3D result = planes[0].closestPoint(otherPoint);
        double distanceSquared = (result-otherPoint).squaredNorm();

        for (int i = 1; i < 6; ++i) {
            Vector3D localResult = planes[i].closestPoint(otherPoint);
            double localDistanceSquared =
                (localResult-otherPoint).squaredNorm();

            if (localDistanceSquared < distanceSquared) {
                result = localResult;
                distanceSquared = localDistanceSquared;
            }
        }

        return result;
    }
    else {
        return clamp(otherPoint, bound.lowerCorner, bound.upperCorner);
    }
}

Vector3D Box3::closestNormalLocal(const Vector3D& otherPoint) const {
    Plane3 planes[6] = { Plane3(Vector3D(1, 0, 0), bound.upperCorner),
                        Plane3(Vector3D(0, 1, 0), bound.upperCorner),
                        Plane3(Vector3D(0, 0, 1), bound.upperCorner),
                        Plane3(Vector3D(-1, 0, 0), bound.lowerCorner),
                        Plane3(Vector3D(0, -1, 0), bound.lowerCorner),
                        Plane3(Vector3D(0, 0, -1), bound.lowerCorner) };

    if (bound.contains(otherPoint)) {
        Vector3D closestNormal = planes[0].normal;
        Vector3D closestPoint = planes[0].closestPoint(otherPoint);
        double minDistanceSquared = (closestPoint - otherPoint).squaredNorm();

        for (int i = 1; i < 6; ++i) {
            Vector3D localClosestPoint = planes[i].closestPoint(otherPoint);
            double localDistanceSquared =
                (localClosestPoint - otherPoint).squaredNorm();

            if (localDistanceSquared < minDistanceSquared) {
                closestNormal = planes[i].normal;
                minDistanceSquared = localDistanceSquared;
            }
        }

        return closestNormal;
    }
    else {
        Vector3D closestPoint =
            clamp(otherPoint, bound.lowerCorner, bound.upperCorner);
        Vector3D closestPointToInputPoint = otherPoint - closestPoint;
        Vector3D closestNormal = planes[0].normal;
        double maxCosineAngle = closestNormal.dot(closestPointToInputPoint);

        for (int i = 1; i < 6; ++i) {
            double cosineAngle = planes[i].normal.dot(closestPointToInputPoint);

            if (cosineAngle > maxCosineAngle) {
                closestNormal = planes[i].normal;
                maxCosineAngle = cosineAngle;
            }
        }

        return closestNormal;
    }
}

bool Box3::intersectsLocal(const Ray3D& ray) const {
    return bound.intersects(ray);
}

SurfaceRayIntersection3 Box3::closestIntersectionLocal(const Ray3D& ray) const {
    SurfaceRayIntersection3 intersection;
    BoundingBoxRayIntersection3D bbRayIntersection =
        bound.closestIntersection(ray);
    intersection.isIntersecting = bbRayIntersection.isIntersecting;
    if (intersection.isIntersecting) {
        intersection.distance = bbRayIntersection.tNear;
        intersection.point = ray.pointAt(bbRayIntersection.tNear);
        intersection.normal = closestNormalLocal(intersection.point);
    }

    return intersection;
}

BoundingBox3D Box3::boundingBoxLocal() const { return bound; }