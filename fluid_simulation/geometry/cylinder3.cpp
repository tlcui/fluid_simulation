#pragma once
#include "cylinder3.h"
#include "plane3.h"
#include "../constants.h"
#include "box3.h"
#include <cmath>

Cylinder3::Cylinder3(const Transform3& transform, bool isNormalFlipped)
    : Surface3(transform, isNormalFlipped) {}

Cylinder3::Cylinder3(const Vector3D& center_, double radius_, double height_,
    const Transform3& transform, bool isNormalFlipped)
    : Surface3(transform, isNormalFlipped),
    center(center_),
    radius(radius_),
    height(height_) {}

Cylinder3::Cylinder3(const Cylinder3& other)
    : Surface3(other),
    center(other.center),
    radius(other.radius),
    height(other.height) {}

Vector3D Cylinder3::closestPointLocal(const Vector3D& otherPoint) const {
    Vector3D r = otherPoint - center;
    //Vector2D rr(std::sqrt(r.x() * r.x() + r.z() * r.z() ), r.y() );
    //Box2 box(Vector2D(-radius, -0.5 * height), Vector2D(radius, 0.5 * height));

    //Vector2D cp = box.closestPoint(rr);
    Vector3D rr(std::sqrt(r.x() * r.x() + r.z() * r.z()), r.y(), 0.);
    Box3 box(Vector3D(-radius, -0.5 * height, 0.), Vector3D(radius, 0.5 * height, 0.));

    Vector3D cp = box.closestPoint(rr);
    double angle = std::atan2(r.z(), r.x());
    return Vector3D(cp.x() * std::cos(angle), cp.y() , cp.x() * std::sin(angle)) +
        center;
}

double Cylinder3::closestDistanceLocal(const Vector3D& otherPoint) const {
    Vector3D r = otherPoint - center;
    //Vector2D rr(std::sqrt(r.x() * r.x() + r.z()  * r.z() ), r.y() );
    //Box2 box(Vector2D(-radius, -0.5 * height), Vector2D(radius, 0.5 * height));

    Vector3D rr(std::sqrt(r.x() * r.x() + r.z() * r.z()), r.y(), 0.);
    Box3 box(Vector3D(-radius, -0.5 * height, 0.), Vector3D(radius, 0.5 * height, 0.));
    return box.closestDistance(rr);
}

Vector3D Cylinder3::closestNormalLocal(const Vector3D& otherPoint) const {
    Vector3D r = otherPoint - center;
    //Vector2D rr(std::sqrt(r.x() * r.x() + r.z()  * r.z() ), r.y() );
    //Box2 box(Vector2D(-radius, -0.5 * height), Vector2D(radius, 0.5 * height));

    Vector3D rr(std::sqrt(r.x() * r.x() + r.z() * r.z()), r.y(), 0.);
    Box3 box(Vector3D(-radius, -0.5 * height, 0.), Vector3D(radius, 0.5 * height, 0.));
    Vector3D cn = box.closestNormal(rr);
    if (cn.y()  > 0) {
        return Vector3D(0, 1, 0);
    }
    else if (cn.y()  < 0) {
        return Vector3D(0, -1, 0);
    }
    else {
        return Vector3D(r.x(), 0, r.z() ).normalized();
    }
}

bool Cylinder3::intersectsLocal(const Ray3D& ray) const {
    // Calculate intersection with infinite cylinder
    // (dx^2 + dz^2)t^2 + 2(ox.dx + oz.dz)t + ox^2 + oz^2 - r^2 = 0
    Vector3D d = ray.direction;
    d.y()  = 0.0;
    Vector3D o = ray.origin - center;
    o.y()  = 0.0;
    double A = d.squaredNorm();
    double B = d.dot(o);
    double C = o.squaredNorm() - radius*radius;

    BoundingBox3D bbox = boundingBox();
    Plane3 upperPlane(Vector3D(0, 1, 0), bbox.upperCorner);
    Plane3 lowerPlane(Vector3D(0, -1, 0), bbox.lowerCorner);

    SurfaceRayIntersection3 upperIntersection =
        upperPlane.closestIntersection(ray);

    SurfaceRayIntersection3 lowerIntersection =
        lowerPlane.closestIntersection(ray);

    // In case the ray does not intersect with infinite cylinder
    if (A < EPSILON_d || B * B - A * C < 0.0) {
        // Check if the ray is inside the infinite cylinder
        Vector3D r = ray.origin - center;
        Vector2D rr(r.x(), r.z() );
        if (rr.squaredNorm() <= radius*radius) {
            if (upperIntersection.isIntersecting ||
                lowerIntersection.isIntersecting) {
                return true;
            }
        }

        return false;
    }

    double t1 = (-B + std::sqrt(B * B - A * C)) / A;
    double t2 = (-B - std::sqrt(B * B - A * C)) / A;
    double tCylinder = t2;

    if (t2 < 0.0) {
        tCylinder = t1;
    }

    Vector3D pointOnCylinder = ray.pointAt(tCylinder);

    if (pointOnCylinder.y()  >= center.y()  - 0.5 * height &&
        pointOnCylinder.y()  <= center.y()  + 0.5 * height) {
        return true;
    }

    if (upperIntersection.isIntersecting) {
        Vector3D r = upperIntersection.point - center;
        r.y()  = 0.0;
        if (r.squaredNorm() <= radius*radius) {
            return true;
        }
    }

    if (lowerIntersection.isIntersecting) {
        Vector3D r = lowerIntersection.point - center;
        r.y()  = 0.0;
        if (r.squaredNorm() <= radius*radius) {
            return true;
        }
    }

    return false;
}

SurfaceRayIntersection3 Cylinder3::closestIntersectionLocal(
    const Ray3D& ray) const {
    SurfaceRayIntersection3 intersection;

    // Calculate intersection with infinite cylinder
    // (dx^2 + dz^2)t^2 + 2(ox.dx + oz.dz)t + ox^2 + oz^2 - r^2 = 0
    Vector3D d = ray.direction;
    d.y()  = 0.0;
    Vector3D o = ray.origin - center;
    o.y()  = 0.0;
    double A = d.squaredNorm();
    double B = d.dot(o);
    double C = o.squaredNorm() - radius*radius;

    BoundingBox3D bbox = boundingBoxLocal();
    Plane3 upperPlane(Vector3D(0, 1, 0), bbox.upperCorner);
    Plane3 lowerPlane(Vector3D(0, -1, 0), bbox.lowerCorner);

    SurfaceRayIntersection3 upperIntersection =
        upperPlane.closestIntersection(ray);

    SurfaceRayIntersection3 lowerIntersection =
        lowerPlane.closestIntersection(ray);

    intersection.distance = std::numeric_limits<double>::max();
    intersection.isIntersecting = false;

    // In case the ray does not intersect with infinite cylinder
    if (A < EPSILON_d || B * B - A * C < 0.0) {
        // Check if the ray is inside the infinite cylinder
        Vector3D r = ray.origin - center;
        Vector2D rr(r.x(), r.z() );
        if (rr.squaredNorm() <= radius*radius) {
            if (upperIntersection.isIntersecting) {
                intersection = upperIntersection;
            }
            if (lowerIntersection.isIntersecting &&
                lowerIntersection.distance < intersection.distance) {
                intersection = lowerIntersection;
            }
        }

        return intersection;
    }

    double t1 = (-B + std::sqrt(B * B - A * C)) / A;
    double t2 = (-B - std::sqrt(B * B - A * C)) / A;
    double tCylinder = t2;

    if (t2 < 0.0) {
        tCylinder = t1;
    }

    Vector3D pointOnCylinder = ray.pointAt(tCylinder);

    if (pointOnCylinder.y()  >= center.y()  - 0.5 * height &&
        pointOnCylinder.y()  <= center.y()  + 0.5 * height) {
        intersection.isIntersecting = true;
        intersection.distance = tCylinder;
        intersection.point = pointOnCylinder;
        intersection.normal = pointOnCylinder - center;
        intersection.normal.y()  = 0.0;
        intersection.normal.normalize();
    }

    if (upperIntersection.isIntersecting) {
        Vector3D r = upperIntersection.point - center;
        r.y()  = 0.0;
        if (r.squaredNorm() > radius*radius) {
            upperIntersection.isIntersecting = false;
        }
        else if (upperIntersection.distance < intersection.distance) {
            intersection = upperIntersection;
        }
    }

    if (lowerIntersection.isIntersecting) {
        Vector3D r = lowerIntersection.point - center;
        r.y()  = 0.0;
        if (r.squaredNorm() > radius * radius) {
            lowerIntersection.isIntersecting = false;
        }
        else if (lowerIntersection.distance < intersection.distance) {
            intersection = lowerIntersection;
        }
    }

    return intersection;
}

BoundingBox3D Cylinder3::boundingBoxLocal() const {
    return BoundingBox3D(center - Vector3D(radius, 0.5 * height, radius),
        center + Vector3D(radius, 0.5 * height, radius));
}