#pragma once
#ifndef RIGID_BODY_COLLIDER3_H_
#define RIGID_BODY_COLLIDER3_H_

#include "collider3.h"

class Rigid_Body_Collider3 final : public Collider3
{
public:
	Vector3D linear_velocity = Vector3D::Zero();
	Vector3D angular_velocity = Vector3D::Zero();

	explicit Rigid_Body_Collider3(const Surface3Ptr& surface);

	Rigid_Body_Collider3(const Surface3Ptr& surface, const Vector3D& linear_velocity_, const Vector3D& angular_velocity_);

	Vector3D get_velocity_at(const Vector3D& point) const override;

};

#endif 