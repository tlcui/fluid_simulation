#pragma once
#ifndef TRANSFORM3_H_
#define TRANSFORM3_H_

#include "bounding_box3.h"
#include "ray3.h"

//! \brief Represents 3-D rigid body transform.
//!
class Transform3 {
public:
    //! Constructs identity transform.
    Transform3();

    //! Constructs a transform with translation and orientation.
    Transform3(const Vector3D& translation, const QuaternionD& orientation);

    //! Returns the translation.
    const Vector3D& translation() const;

    //! Sets the traslation.
    void setTranslation(const Vector3D& translation);

    //! Returns the orientation.
    const QuaternionD& orientation() const;

    //! Sets the orientation.
    void setOrientation(const QuaternionD& orientation);

    //! Transforms a point in world coordinate to the local frame.
    Vector3D toLocal(const Vector3D& pointInWorld) const;

    //! Transforms a direction in world coordinate to the local frame.
    Vector3D toLocalDirection(const Vector3D& dirInWorld) const;

    //! Transforms a ray in world coordinate to the local frame.
    Ray3D toLocal(const Ray3D& rayInWorld) const;

    //! Transforms a bounding box in world coordinate to the local frame.
    BoundingBox3D toLocal(const BoundingBox3D& bboxInWorld) const;

    //! Transforms a point in local space to the world coordinate.
    Vector3D toWorld(const Vector3D& pointInLocal) const;

    //! Transforms a direction in local space to the world coordinate.
    Vector3D toWorldDirection(const Vector3D& dirInLocal) const;

    //! Transforms a ray in local space to the world coordinate.
    Ray3D toWorld(const Ray3D& rayInLocal) const;

    //! Transforms a bounding box in local space to the world coordinate.
    BoundingBox3D toWorld(const BoundingBox3D& bboxInLocal) const;

private:
    Vector3D _translation;
    QuaternionD _orientation;
    Matrix3D _orientationMat3;
    Matrix3D _inverseOrientationMat3;
};

inline Transform3::Transform3()
{
    setTranslation(Vector3D::Zero());
    //setOrientation(QuaternionD::Identity());
    _orientation = QuaternionD::Identity();
    _orientationMat3 = Matrix3D::Identity();
    _inverseOrientationMat3 = Matrix3D::Identity();
}

inline Transform3::Transform3(
    const Vector3D& translation,
    const QuaternionD& orientation) {
    setTranslation(translation);
    setOrientation(orientation);
}

inline const Vector3D& Transform3::translation() const {
    return _translation;
}

inline void Transform3::setTranslation(const Vector3D& translation) {
    _translation = translation;
}

inline const QuaternionD& Transform3::orientation() const {
    return _orientation;
}

inline void Transform3::setOrientation(const QuaternionD& orientation) {
    _orientation = orientation;
    _orientationMat3 = orientation.normalized().toRotationMatrix();
    _inverseOrientationMat3 = orientation.inverse().normalized().toRotationMatrix();
}

inline Vector3D Transform3::toLocal(const Vector3D& pointInWorld) const {
    return _inverseOrientationMat3 * (pointInWorld - _translation);
}

inline Vector3D Transform3::toLocalDirection(const Vector3D& dirInWorld) const {
    return _inverseOrientationMat3 * dirInWorld;
}

inline Ray3D Transform3::toLocal(const Ray3D& rayInWorld) const {
    return Ray3D(
        toLocal(rayInWorld.origin),
        toLocalDirection(rayInWorld.direction));
}

inline BoundingBox3D Transform3::toLocal(
    const BoundingBox3D& bboxInWorld) const {
    BoundingBox3D bboxInLocal;
    for (int i = 0; i < 8; ++i) {
        auto cornerInLocal = toLocal(bboxInWorld.corner(i));
        bboxInLocal.lowerCorner
            = min(bboxInLocal.lowerCorner, cornerInLocal);
        bboxInLocal.upperCorner
            = max(bboxInLocal.upperCorner, cornerInLocal);
    }
    return bboxInLocal;
}

inline Vector3D Transform3::toWorld(const Vector3D& pointInLocal) const {
    return (_orientationMat3 * pointInLocal) + _translation;
}

inline Vector3D Transform3::toWorldDirection(
    const Vector3D& dirInLocal) const {
    return _orientationMat3 * dirInLocal;
}

inline Ray3D Transform3::toWorld(const Ray3D& rayInLocal) const {
    return Ray3D(
        toWorld(rayInLocal.origin),
        toWorldDirection(rayInLocal.direction));
}

inline BoundingBox3D Transform3::toWorld(
    const BoundingBox3D& bboxInLocal) const {
    BoundingBox3D bboxInWorld;
    for (int i = 0; i < 8; ++i) {
        auto cornerInWorld = toWorld(bboxInLocal.corner(i));
        bboxInWorld.lowerCorner
            = min(bboxInWorld.lowerCorner, cornerInWorld);
        bboxInWorld.upperCorner
            = max(bboxInWorld.upperCorner, cornerInWorld);
    }
    return bboxInWorld;
}

#endif 