#include "pcisph_solver3.h"
#include "bcc_lattice_point_generator.h"
#include <iostream>

const double default_timestep_limit_scale = 5.0;

Pcisph_Solver3::Pcisph_Solver3()
{
	set_timestep_limit_scale(default_timestep_limit_scale);
}

Pcisph_Solver3::Pcisph_Solver3(double target_density, double target_spacing, double relative_kernel_radius):
	Sph_Solver3(target_density, target_spacing, relative_kernel_radius)
{
	set_timestep_limit_scale(default_timestep_limit_scale);
}

Pcisph_Solver3::~Pcisph_Solver3()
{
}

double Pcisph_Solver3::get_max_density_error_ratio() const
{
	return _max_density_error_ratio;
}

void Pcisph_Solver3::set_max_density_error_ratio(double ratio)
{
	_max_density_error_ratio = std::max(ratio, 0.0);
}

unsigned int Pcisph_Solver3::get_max_number_of_iterations() const
{
	return _max_number_of_iterations;
}

void Pcisph_Solver3::set_max_number_of_iterations(unsigned int n)
{
	_max_number_of_iterations = n;
}

void Pcisph_Solver3::accumulate_pressure_force(double timestep_in_seconds)
{
	auto particles = get_sph_system_data();
	const size_t number_of_particles = particles->get_number_of_particles();
	const double delta = compute_delta(timestep_in_seconds);
	const double target_density = particles->get_target_density();
	const double mass = particles->get_particle_mass();
	auto& x = particles->get_positions();
	auto& v = particles->get_velocities();
	auto& p = particles->get_pressures();
	auto& d = particles->get_densities();
	auto& f = particles->get_forces();

	//predicted density
	std::vector<double> density_star(number_of_particles, 0.);

	Sph_Std_Kernel3 kernel(particles->get_kernel_radius());

	//initialize buffers
	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				p[i] = 0.;
				_pressure_forces[i] = Vector3D::Zero();
				_density_errors[i] = 0.;
				density_star[i] = d[i];
			}
		});

	double max_density_error;
	double density_error_ratio = 0.;

	for (unsigned int k = 0; k < _max_number_of_iterations; ++k)
	{
		//predict velocity and position
		tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
			{
				for (size_t i = r.begin(); i != r.end(); ++i)
				{
					_temp_velocities[i] = (v[i] + timestep_in_seconds * (f[i] + _pressure_forces[i]) / mass);
					_temp_positions[i] = x[i] + timestep_in_seconds * _temp_velocities[i];
				}
			});

		//resolve collisions
		resolve_collision(_temp_positions, _temp_velocities);

		//compute pressure from density error
		tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
			{
				for (size_t i = r.begin(); i != r.end(); ++i)
				{
					double weight_sum = 0.;
					const auto& neighbors = particles->get_neighbor_lists()[i];

					for (size_t j : neighbors)
					{
						double dist = (_temp_positions[i] - _temp_positions[j]).norm();
						weight_sum += kernel(dist);
					}
					weight_sum += kernel(0);

					double density = mass * weight_sum;
					double density_error = density - target_density;
					double pressure = delta * density_error;

					if (pressure < 0.0)
					{
						pressure *= get_negative_pressure_scale();
						density_error *= get_negative_pressure_scale();
					}

					p[i] += pressure;  //iteration

					density_star[i] = density;
					_density_errors[i] = density_error;
				}
			});

		//compute pressure gradient force
		//we have to clear pressure forces here, because forces will accumulate in Sph_Solver3::accumulate_pressure_force()
		//note that we are computing forces here, not accumulating forces
		_pressure_forces.resize(number_of_particles, Vector3D::Zero()); 
		Sph_Solver3::accumulate_pressure_force(x, density_star, p, _pressure_forces);

		//compute max density error
		max_density_error = 0.;
		for (size_t i = 0; i < number_of_particles; ++i)
		{
			max_density_error = absmax(max_density_error, _density_errors[i]);
		}
		density_error_ratio = max_density_error / target_density;

		if (std::fabs(density_error_ratio) < _max_density_error_ratio)
		{
			break;
		}
	}

	//accumulate pressure force
	tbb::parallel_for(tbb::blocked_range<size_t>(0, number_of_particles), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				f[i] += _pressure_forces[i];
			}
		});
}

void Pcisph_Solver3::on_timestep_start(double timestep_in_seconds)
{
	Sph_Solver3::on_timestep_start(timestep_in_seconds);

	// allocate temp buffers
	size_t number_of_particles = get_sph_system_data()->get_number_of_particles();
	_temp_positions.resize(number_of_particles, Vector3D::Zero());
	_temp_velocities.resize(number_of_particles, Vector3D::Zero());
	_pressure_forces.resize(number_of_particles, Vector3D::Zero());
	_density_errors.resize(number_of_particles, 0.);
}

double Pcisph_Solver3::compute_delta(double timestep_in_seconds)
{
	auto particles = get_sph_system_data();
	const double kernel_radius = particles->get_kernel_radius();

	std::vector<Vector3D> points;
	BccLatticePointGenerator pointsGenerator;
	Vector3D origin = Vector3D::Zero();
	BoundingBox3D sampleBound(origin, origin);
	sampleBound.expand(1.5 * kernel_radius);

	pointsGenerator.generate(sampleBound, particles->get_target_spacing(), &points);

	Sph_Spiky_Kernel3 kernel(kernel_radius);

	double denom = 0;
	Vector3D denom1 = Vector3D::Zero();
	double denom2 = 0;

	for (size_t i = 0; i < points.size(); ++i) 
	{
		const Vector3D& point = points[i];
		double distanceSquared = point.squaredNorm();

		if (distanceSquared < kernel_radius * kernel_radius) {
			double distance = std::sqrt(distanceSquared);
			Vector3D direction = Vector3D::Zero();
			if (distance > 0.0)
			{
				direction = (point / distance);
			} 
			else
			{
				direction = Vector3D::Zero();
			}

			// grad(Wij)
			Vector3D gradWij = kernel.get_gradient(distance, direction);
			denom1 += gradWij;
			denom2 += gradWij.dot(gradWij);
		}
	}

	denom += -denom1.dot(denom1) - denom2;

	return (std::fabs(denom) > 0.0) ?
		-1 / (compute_beta(timestep_in_seconds) * denom) : 0;
}

double Pcisph_Solver3::compute_beta(double timestep_in_seconds)
{
	auto particles = get_sph_system_data();
	const double mass = particles->get_particle_mass();
	const double target_density = particles->get_target_density();

	return 2.0 * timestep_in_seconds * timestep_in_seconds * mass * mass / (target_density * target_density);
}
