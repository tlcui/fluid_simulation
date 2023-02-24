#include "sph_solver3.h"
#include <iostream>

static constexpr double timestep_limit_by_speed_factor = 0.4;
static constexpr double timestep_limit_by_force_factor = 0.25;

Sph_Solver3::Sph_Solver3()
{
	set_particle_system_data(std::make_shared<Sph_System_Data3>());
	set_is_subtimestep_fixed(false);
}

Sph_Solver3::Sph_Solver3(double target_density, double target_spacing, double relative_kernel_radius)
{
	auto sph_particles = std::make_shared<Sph_System_Data3>();
	set_particle_system_data(sph_particles);
	sph_particles->set_target_density(target_density);
	sph_particles->set_target_spacing(target_spacing);
	sph_particles->set_relative_kernel_radius(relative_kernel_radius);
	set_is_subtimestep_fixed(false);
}

Sph_Solver3::~Sph_Solver3()
{
}

double Sph_Solver3::get_eos_exponent() const
{
	return _eos_exponent;
}

void Sph_Solver3::set_eos_exponent(double eos_exponent)
{
	_eos_exponent = std::max(eos_exponent, 1.0);
}

double Sph_Solver3::get_negative_pressure_scale() const
{
	return _negative_pressure_scale;
}

void Sph_Solver3::set_negative_pressure_scale(double negative_pressure_scale)
{
	_negative_pressure_scale = clamp(negative_pressure_scale, 0.0, 1.0);
}

double Sph_Solver3::get_viscosity_coefficient() const
{
	return _viscosity_coefficient;
}

void Sph_Solver3::set_viscosity_coefficient(double viscosity_coefficient)
{
	_viscosity_coefficient = std::max(viscosity_coefficient, 0.0);
}

double Sph_Solver3::get_pseudo_viscosity_coefficient() const
{
	return _pseudo_viscosity_coefficient;
}

void Sph_Solver3::set_pseudo_viscosity_coefficient(double viscosity_coefficient)
{
	_pseudo_viscosity_coefficient = std::max(viscosity_coefficient, 0.0);
}

double Sph_Solver3::get_speed_of_sound() const
{
	return _speed_of_sound;
}

void Sph_Solver3::set_speed_of_sound(double speed_of_sound)
{
	_speed_of_sound = std::max(speed_of_sound, EPSILON_d);
}

double Sph_Solver3::get_timestep_limit_scale() const
{
	return _timestep_limit_scale;
}

void Sph_Solver3::set_timestep_limit_scale(double timestep_limit_scale)
{
	_timestep_limit_scale = std::max(timestep_limit_scale, 0.0);
}

Sph_System_Data3_Ptr Sph_Solver3::get_sph_system_data() const
{
	return std::dynamic_pointer_cast<Sph_System_Data3>(get_particle_system_data());
}

unsigned int Sph_Solver3::get_number_of_subtimesteps(double timestep_in_seconds) const
{
	auto particles = get_sph_system_data();
	size_t number_of_particles = particles->get_number_of_particles();
	auto& f = particles->get_forces();

	const double kernel_radius = particles->get_kernel_radius();
	const double mass = particles->get_particle_mass();

	double max_force_magnitude = 0.;

	for (size_t i = 0; i < number_of_particles; ++i)
	{
		max_force_magnitude = std::max(max_force_magnitude, f[i].norm());
	}

	double timestep_limit_by_speed = timestep_limit_by_speed_factor * kernel_radius / get_speed_of_sound();
	double timestep_limit_by_force = timestep_limit_by_force_factor * std::sqrt(kernel_radius * mass / max_force_magnitude);

	double desired_timestep = _timestep_limit_scale * std::min(timestep_limit_by_force, timestep_limit_by_speed);

	return static_cast<unsigned int>(std::ceil(timestep_in_seconds / desired_timestep));
}

void Sph_Solver3::accumulate_forces(double timestep_in_seconds)
{
	accumulate_non_pressure_forces(timestep_in_seconds);
	accumulate_pressure_force(timestep_in_seconds);
}

void Sph_Solver3::on_timestep_start(double timestep_in_seconds)
{
	auto particles = get_sph_system_data();
	particles->build_neighbor_searcher();
	particles->build_neighbor_lists();
	particles->update_densities();
}

void Sph_Solver3::on_timestep_end(double timestep_in_seconds)
{
	compute_pseudo_viscosity(timestep_in_seconds);

}

void Sph_Solver3::accumulate_non_pressure_forces(double timestep_in_seconds)
{
	Particle_System_Solver3::accumulate_forces(timestep_in_seconds);
	accumulate_viscosity_force();
}

void Sph_Solver3::accumulate_pressure_force(double timestep_in_seconds)
{
	auto particles = get_sph_system_data();
	auto& x = particles->get_positions();
	auto& p = particles->get_pressures();
	auto& d = particles->get_densities();
	auto& f = particles->get_forces();

	compute_pressure();
	accumulate_pressure_force(x, d, p, f);
}

void Sph_Solver3::compute_pressure()
{
	auto particles = get_sph_system_data();
	size_t number_of_particles = particles->get_number_of_particles();
	auto& d = particles->get_densities();
	auto& p = particles->get_pressures();

	const double target_density = particles->get_target_density();
	const double eos_scale = target_density * _speed_of_sound * _speed_of_sound;

	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				p[i] = compute_pressure_from_eos(d[i], target_density, eos_scale, get_eos_exponent(), get_negative_pressure_scale());
			}
		});
}

void Sph_Solver3::accumulate_pressure_force(const std::vector<Vector3D>& positions, const std::vector<double>& densities, const std::vector<double>& pressures, std::vector<Vector3D>& pressure_forces)
{
	auto particles = get_sph_system_data();
	size_t number_of_particles = particles->get_number_of_particles();
	const double mass = particles->get_particle_mass();
	const Sph_Spiky_Kernel3 kernel(particles->get_kernel_radius());

	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				const auto& neighbors = particles->get_neighbor_lists()[i];
				for (size_t j : neighbors)
				{
					double dist = (positions[i] - positions[j]).norm();
					if (dist > 0.0)
					{
						Vector3D dir((positions[j] - positions[i]) / dist);
						pressure_forces[i] -= mass * mass * (pressures[i] / (densities[i] * densities[i]) + pressures[j] / (densities[j] * densities[j])) * kernel.get_gradient(dist, dir);
					}
				}
			}
		});
}

void Sph_Solver3::accumulate_viscosity_force()
{
	auto particles = get_sph_system_data();
	auto& x = particles->get_positions();
	auto& v = particles->get_velocities();
	auto& d = particles->get_densities();
	auto& f = particles->get_forces();
	size_t number_of_particles = particles->get_number_of_particles();

	const double mass = particles->get_particle_mass();
	const Sph_Spiky_Kernel3 kernel(particles->get_kernel_radius());

	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				const auto& neighbors = particles->get_neighbor_lists()[i];
				for (size_t j : neighbors)
				{
					double dist = (x[i] - x[j]).norm();
					f[i] += (get_viscosity_coefficient() * mass * mass * (v[j] - v[i]) / d[j] * kernel.get_second_derivative(dist));
				}
			}
		});
}

void Sph_Solver3::compute_pseudo_viscosity(double timestep_in_seconds)
{
	auto particles = get_sph_system_data();
	auto& x = particles->get_positions();
	auto& v = particles->get_velocities();
	auto& d = particles->get_densities();
	size_t number_of_particles = particles->get_number_of_particles();

	const double mass = particles->get_particle_mass();
	const Sph_Spiky_Kernel3 kernel(particles->get_kernel_radius());

	std::vector<Vector3D> smoothed_velocities(number_of_particles, Vector3D::Zero());

	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				double weight_sum = 0.0;
				Vector3D smoothed_velocity (Vector3D::Zero());
				const auto& neighbors = particles->get_neighbor_lists()[i];
				for (size_t j : neighbors)
				{
					double dist = (x[i] - x[j]).norm();
					double wj = mass / d[j] * kernel(dist);
					weight_sum += wj;
					smoothed_velocity =  smoothed_velocity + wj * v[j];
				}

				double wi = mass / d[i];
				weight_sum += wi;
				smoothed_velocity += wi * v[i];

				if (weight_sum > 0.0)
				{
					smoothed_velocity /= weight_sum;
				}

				smoothed_velocities[i] = smoothed_velocity;
			}
		});

	double factor = timestep_in_seconds * _pseudo_viscosity_coefficient;
	factor = clamp(factor, 0.0, 1.0);

	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				v[i] = lerp(v[i], smoothed_velocities[i], factor);
			}
		});
}

inline double compute_pressure_from_eos(
	double density,
	double targetDensity,
	double eosScale,
	double eosExponent,
	double negativePressureScale) {
	// See Murnaghan-Tait equation of state from
	// https://en.wikipedia.org/wiki/Tait_equation
	double p = eosScale / eosExponent
		* (std::pow((density / targetDensity), eosExponent) - 1.0);

	// Negative pressure scaling
	if (p < 0) {
		p *= negativePressureScale;
	}

	return p;
}