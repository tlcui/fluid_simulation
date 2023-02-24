#include "rigid_body_collider3.h"

Rigid_Body_Collider3::Rigid_Body_Collider3(const Surface3Ptr& surface)
{
	set_surface(surface);
}

Rigid_Body_Collider3::Rigid_Body_Collider3(const Surface3Ptr& surface, const Vector3D& linear_velocity_, const Vector3D& angular_velocity_):
	linear_velocity(linear_velocity_), angular_velocity(angular_velocity_)
{
	set_surface(surface);
}

Vector3D Rigid_Body_Collider3::get_velocity_at(const Vector3D& point) const
{
	Vector3D r = point - get_surface()->transform.translation();
	return linear_velocity + angular_velocity.cross(r);
}
