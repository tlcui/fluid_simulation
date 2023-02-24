#include "particle_system_solver3.h"
#include "constant_vector_field3.h"
#include "../common_utils.h"
#include <iostream>

Particle_System_Solver3::Particle_System_Solver3() : Particle_System_Solver3(1e-3, 1e-3)
{
}

Particle_System_Solver3::Particle_System_Solver3(double radius, double mass)
{
	_particle_system_data = std::make_shared<Particle_System_Data3>();
	_particle_system_data->set_particle_radius(radius);
	_particle_system_data->set_particle_mass(mass);
	_wind = std::make_shared<Constant_Vector_Field3>(Vector3D::Zero());
}

Particle_System_Solver3::~Particle_System_Solver3()
{
}

double Particle_System_Solver3::get_drag_coefficient() const
{
	return _drag_coefficient;
}

void Particle_System_Solver3::set_drag_coefficient(double new_drag_coefficient)
{
	_drag_coefficient = std::max(new_drag_coefficient, 0.0);
}

double Particle_System_Solver3::get_restitution_coefficient() const
{
	return _restitution_coefficient;
}

void Particle_System_Solver3::set_restitution_coefficient(double new_restitution_coefficient)
{
	_restitution_coefficient = clamp(new_restitution_coefficient, 0.0, 1.0);
}

const Vector3D& Particle_System_Solver3::get_gravity() const
{
	return _gravity;
}

void Particle_System_Solver3::set_gravity(const Vector3D& new_gravity)
{
	_gravity = new_gravity;
}

const Particle_System_Data3_Ptr& Particle_System_Solver3::get_particle_system_data() const
{
	return _particle_system_data;
}

const Vector_Field3_Ptr& Particle_System_Solver3::get_wind() const
{
	return _wind;
}

void Particle_System_Solver3::set_wind(const Vector_Field3_Ptr& new_wind)
{
	_wind = new_wind;
}

const Collider3_Ptr& Particle_System_Solver3::get_collider() const
{
	return _collider;
}

void Particle_System_Solver3::set_collider(const Collider3_Ptr& new_collider)
{
	_collider = new_collider;
}

const Particle_Emitter3_Ptr& Particle_System_Solver3::get_emitter() const
{
	return _emitter;
}

void Particle_System_Solver3::set_emitter(const Particle_Emitter3_Ptr& new_emitter)
{
	_emitter = new_emitter;
	_emitter->setTarget(_particle_system_data);
}

void Particle_System_Solver3::on_initialize()
{
	update_collider(0.0);
	update_emitter(0.0);
}

void Particle_System_Solver3::on_advance_timestep(double timestep_in_seconds)
{
	timestep_start(timestep_in_seconds);
	accumulate_forces(timestep_in_seconds);
	time_integration(timestep_in_seconds);
	resolve_collision();
	timestep_end(timestep_in_seconds);
}

void Particle_System_Solver3::accumulate_forces(double timestep_in_seconds)
{
	accumulate_external_forces();
}

void Particle_System_Solver3::on_timestep_start(double timestep_in_seconds)
{
}

void Particle_System_Solver3::on_timestep_end(double timestep_in_seconds)
{
}

void Particle_System_Solver3::resolve_collision()
{
	resolve_collision(_new_positions, _new_velocities);
}

void Particle_System_Solver3::resolve_collision(std::vector<Vector3D>& new_positions, std::vector<Vector3D>& new_velocities)
{
	size_t number_of_particles = _particle_system_data->get_number_of_particles();
	const double radius = _particle_system_data->get_particle_radius();

	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				_collider->resolve_collision(radius, _restitution_coefficient, &_new_positions[i], &_new_velocities[i]);
			}
		});
}

void Particle_System_Solver3::set_particle_system_data(const Particle_System_Data3_Ptr& new_particles)
{
	_particle_system_data = new_particles;
}

void Particle_System_Solver3::timestep_start(double timestep_in_seconds)
{
	auto& forces = _particle_system_data->get_forces();
	size_t n = _particle_system_data->get_number_of_particles();

	//clear forces
	tbb::parallel_for(tbb::blocked_range<size_t>(0, forces.size()), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				forces[i] = Vector3D::Zero();
			}
		});
	//forces.resize(n, Vector3D::Zero());

	update_collider(timestep_in_seconds);
	update_emitter(timestep_in_seconds);

	_new_positions.resize(n, Vector3D::Zero());
	_new_velocities.resize(n, Vector3D::Zero());
	

	on_timestep_start(timestep_in_seconds);
}

void Particle_System_Solver3::timestep_end(double timestep_in_seconds)
{
	size_t n = _particle_system_data->get_number_of_particles();

	auto& positions = _particle_system_data->get_positions();
	auto& velocities = _particle_system_data->get_velocities();
	auto& positions_float = _particle_system_data->get_positions_float();

	tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				positions[i] = _new_positions[i];
				velocities[i] = _new_velocities[i];
				positions_float[3 * i] = _new_positions[i].x();
				positions_float[3 * i + 1] = _new_positions[i].y();
				positions_float[3 * i + 2] = _new_positions[i].z();
			}
		});

	on_timestep_end(timestep_in_seconds);
}

void Particle_System_Solver3::accumulate_external_forces()
{
	size_t n = _particle_system_data->get_number_of_particles();
	auto& forces = _particle_system_data->get_forces();
	auto& velocities = _particle_system_data->get_velocities();
	auto& positions = _particle_system_data->get_positions();
	const double mass = _particle_system_data->get_particle_mass();

	tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				Vector3D force = mass * _gravity;
				Vector3D relative_vel = velocities[i] - _wind->sample(positions[i]);
				force += -_drag_coefficient * relative_vel;

				forces[i] += force;
			}
		});
}

void Particle_System_Solver3::time_integration(double timestep_in_seconds)
{
	size_t n = _particle_system_data->get_number_of_particles();
	auto& forces = _particle_system_data->get_forces();
	auto& velocities = _particle_system_data->get_velocities();
	auto& positions = _particle_system_data->get_positions();
	const double mass = _particle_system_data->get_particle_mass();

	tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				Vector3D& new_velocity = _new_velocities[i];
				new_velocity = velocities[i] + timestep_in_seconds * forces[i] / mass;

				Vector3D& new_position = _new_positions[i];
				new_position = positions[i] + timestep_in_seconds * new_velocity;
			}
		});
}

void Particle_System_Solver3::update_collider(double timestep_in_seconds)
{
	if (_collider != nullptr)
	{
		_collider->update(get_current_time_in_seconds(), timestep_in_seconds);
	}
}

void Particle_System_Solver3::update_emitter(double timestep_in_seconds)
{
	// this function will be called only once during the entire simulation
	// because our emitter is one-shot
	if (_emitter != nullptr)
	{
		_emitter->update(get_current_time_in_seconds(), timestep_in_seconds);
	}
}
