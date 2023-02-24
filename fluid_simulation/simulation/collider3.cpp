#include "collider3.h"

Collider3::Collider3()
{
}

Collider3::~Collider3()
{
}

double Collider3::get_friction_coefficient() const
{
	return _friction_coefficient;
}

void Collider3::set_friction_coefficient(double new_coefficient)
{
	_friction_coefficient = std::max(new_coefficient, 0.0);
}

const Surface3Ptr& Collider3::get_surface() const
{
	return _surface;
}

void Collider3::update(double current_time_in_seconds, double time_interval_in_seconds)
{
	if (!_surface->isValidGeometry()) {
		return;
	}

	_surface->updateQueryEngine();

	if (_on_update_callback) {
		_on_update_callback(this, current_time_in_seconds, time_interval_in_seconds);
	}
}

void Collider3::set_on_begin_update_callback(const On_Begin_Update_Callback& callback)
{
	_on_update_callback = callback;
}

void Collider3::resolve_collision(double radius, double restitutionCoefficient, Vector3D* position, Vector3D* velocity)
{
	if (!_surface->isValidGeometry()) {
		return;
	}

	Collider_Query_Result collider_point;
	get_closest_point(_surface, *position, &collider_point);

	if (is_penetrating(collider_point, *position, radius))
	{
		// Target point is the closest non-penetrating position from the new position.
		Vector3D target_normal = collider_point.normal;
		Vector3D target_point = collider_point.point + radius * target_normal;
		Vector3D collider_vel_at_target_point = collider_point.velocity;

		Vector3D relative_vel = *velocity - collider_vel_at_target_point;
		double normal_dot_relative_vel = target_normal.dot(relative_vel);
		Vector3D relative_vel_N = normal_dot_relative_vel * target_normal;
		Vector3D relative_vel_T = relative_vel - relative_vel_N;

		// Check if the velocity is facing opposite direction of the surface normal
		if (normal_dot_relative_vel < 0.)
		{
			Vector3D delta_relative_vel_N = (-1.0 - restitutionCoefficient) * relative_vel_N;
			relative_vel_N *= -restitutionCoefficient;

			if (relative_vel_T.squaredNorm() > 0.)
			{
				double friction_scale = std::max(1.0 - _friction_coefficient * delta_relative_vel_N.norm() / relative_vel_T.norm(), 0.0);
				relative_vel_T *= friction_scale;
			}

			*velocity = relative_vel_N + relative_vel_T + collider_vel_at_target_point;
		}

		*position = target_point;
	}
}

void Collider3::set_surface(const Surface3Ptr& newSurface)
{
	_surface = newSurface;
}

void Collider3::get_closest_point(const Surface3Ptr& surface, const Vector3D& query_point, Collider_Query_Result* result) const
{
	result->distance = surface->closestDistance(query_point);
	result->point = surface->closestPoint(query_point);
	result->normal = surface->closestNormal(query_point);
	result->velocity = get_velocity_at(query_point);
}

bool Collider3::is_penetrating(const Collider_Query_Result& collider_point, const Vector3D& position, double radius)
{
	return _surface->isInside(position) || collider_point.distance < radius;
}
