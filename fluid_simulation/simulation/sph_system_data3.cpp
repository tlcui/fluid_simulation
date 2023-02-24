#include "sph_system_data3.h"
#include "sph_kernels3.h"
#include "bcc_lattice_point_generator.h"
#include <iostream>

Sph_System_Data3::Sph_System_Data3(): Sph_System_Data3(0)
{
}

Sph_System_Data3::Sph_System_Data3(size_t number_of_particles): Particle_System_Data3(number_of_particles)
{
	_density_index = add_scalar_data();
	_pressure_index = add_scalar_data();

	set_target_spacing(_target_spacing);
}

Sph_System_Data3::Sph_System_Data3(const Sph_System_Data3& other)
{
	set(other);
}

Sph_System_Data3::~Sph_System_Data3()
{
}

void Sph_System_Data3::set(const Sph_System_Data3& other)
{
	Particle_System_Data3::set(other);

	_target_density = other._target_density;
	_target_spacing = other._target_spacing;
	_kernel_radius_over_target_spacing = other._kernel_radius_over_target_spacing;
	_kernel_radius = other._kernel_radius;
	_density_index = other._density_index;
	_pressure_index = other._pressure_index;
}

Sph_System_Data3& Sph_System_Data3::operator=(const Sph_System_Data3& other)
{
	set(other);
	return *this;
}

double Sph_System_Data3::get_target_spacing() const
{
	return _target_spacing;
}

void Sph_System_Data3::set_target_spacing(double spacing)
{
	Particle_System_Data3::set_particle_radius(spacing);
	_target_spacing = spacing;
	_kernel_radius = _kernel_radius_over_target_spacing * _target_spacing;

	compute_mass();
}

void Sph_System_Data3::set_particle_radius(double new_radius)
{
	set_target_spacing(new_radius);
}

void Sph_System_Data3::set_relative_kernel_radius(double relativeRadius)
{
	_kernel_radius_over_target_spacing = relativeRadius;
	_kernel_radius = _kernel_radius_over_target_spacing * _target_spacing;

	compute_mass();
}

double Sph_System_Data3::get_relative_kernel_radius() const
{
	return _kernel_radius_over_target_spacing;
}

void Sph_System_Data3::set_particle_mass(double new_mass)
{
	//double ratio = new_mass / get_particle_mass();
	//_target_density *= ratio;

	Particle_System_Data3::set_particle_mass(new_mass);
}

const std::vector<double>& Sph_System_Data3::get_densities() const
{
	return scalar_data_at(_density_index);
}

std::vector<double>& Sph_System_Data3::get_densities()
{
	return scalar_data_at(_density_index);
}

double Sph_System_Data3::get_target_density() const
{
	return _target_density;
}

void Sph_System_Data3::set_target_density(double target_density)
{
	_target_density = target_density;
	compute_mass();
}

void Sph_System_Data3::update_densities()
{
	auto& p = get_positions();
	auto& d = get_densities();
	const double m = get_particle_mass();

	tbb::parallel_for(tbb::blocked_range<size_t>(0, get_number_of_particles()), [&](const tbb::blocked_range<size_t>& r)
		{
			for (size_t i = r.begin(); i != r.end(); ++i)
			{
				double sum = get_sum_of_kernel_nearby(p[i]);
				d[i] = m * sum;
			}
		});
}

const std::vector<double>& Sph_System_Data3::get_pressures() const
{
	return scalar_data_at(_pressure_index);
}

std::vector<double>& Sph_System_Data3::get_pressures()
{
	return scalar_data_at(_pressure_index);
}

double Sph_System_Data3::get_kernel_radius() const
{
	return _kernel_radius;
}

void Sph_System_Data3::set_kernel_radius(double kernel_radius)
{
	_kernel_radius = kernel_radius;
	_target_spacing = kernel_radius / _kernel_radius_over_target_spacing;

	compute_mass();
}

double Sph_System_Data3::get_sum_of_kernel_nearby(const Vector3D& origin) const
{
	double sum = 0. ;
	Sph_Std_Kernel3 kernel(_kernel_radius);
	get_neighbor_searcher()->for_each_nearby_point(origin, _kernel_radius, [&](size_t, const Vector3D& position) 
		{
			double dist = (origin - position).norm();
			sum += kernel(dist);
		});
	return sum;
}

double Sph_System_Data3::interpolate(const Vector3D& origin, const std::vector<double>& values) const
{
	double sum = 0.;
	auto& d = get_densities();
	Sph_Std_Kernel3 kernel(_kernel_radius);
	const double m = get_particle_mass();

	get_neighbor_searcher()->for_each_nearby_point(origin, _kernel_radius, [&](size_t i, const Vector3D& position)
		{
			double dist = (origin - position).norm();
			double weight = m / d[i] * kernel(dist);
			sum += (values[i] * weight);
		});
	return sum;
}

Vector3D Sph_System_Data3::interpolate(const Vector3D& origin, const std::vector<Vector3D>& values) const
{
	Vector3D sum = Vector3D::Zero();
	auto& d = get_densities();
	Sph_Std_Kernel3 kernel(_kernel_radius);
	const double m = get_particle_mass();

	get_neighbor_searcher()->for_each_nearby_point(origin, _kernel_radius, [&](size_t i, const Vector3D& position)
		{
			double dist = (origin - position).norm();
			double weight = m / d[i] * kernel(dist);
			sum += (values[i] * weight);
		});
	return sum;
}

Vector3D Sph_System_Data3::get_gradient_at(size_t i, const std::vector<double>& values) const
{
	Vector3D sum = Vector3D::Zero();
	auto& p = get_positions();
	auto& d = get_densities();
	const auto& neighbors = get_neighbor_lists()[i];
	Vector3D origin = p[i];
	Sph_Spiky_Kernel3 kernel(_kernel_radius);
	const double m = get_particle_mass();

	for (size_t j : neighbors)
	{
		Vector3D position = p[j];
		double dist = (origin - position).norm();
		if (dist > 0.0)
		{
			Vector3D dir = (position - origin) / dist;
			sum += d[i] * m * (values[i] / (d[i] * d[i]) + values[j] / (d[j] * d[j])) * kernel.get_gradient(dist, dir);
		}
	}

	return sum;
}

double Sph_System_Data3::get_laplacian_at(size_t i, const std::vector<double>& values) const
{
	double sum = 0.;
	auto& p = get_positions();
	auto& d = get_densities();
	const auto& neighbors = get_neighbor_lists()[i];
	Vector3D origin = p[i];
	Sph_Spiky_Kernel3 kernel(_kernel_radius);
	const double m = get_particle_mass();

	for (size_t j : neighbors)
	{
		Vector3D position = p[j];
		double dist = (origin - position).norm();
		
		sum += m * (values[j] - values[i]) / d[j] * kernel.get_second_derivative(dist);
	}

	return sum;
}

Vector3D Sph_System_Data3::get_laplacian_at(size_t i, const std::vector<Vector3D>& values) const
{
	Vector3D sum = Vector3D::Zero();
	auto& p = get_positions();
	auto& d = get_densities();
	const auto& neighbors = get_neighbor_lists()[i];
	Vector3D origin = p[i];
	Sph_Spiky_Kernel3 kernel(_kernel_radius);
	const double m = get_particle_mass();

	for (size_t j : neighbors)
	{
		Vector3D position = p[j];
		double dist = (origin - position).norm();

		sum += m * (values[j] - values[i]) / d[j] * kernel.get_second_derivative(dist);
	}

	return sum;
}

void Sph_System_Data3::build_neighbor_searcher()
{
	Particle_System_Data3::build_neighbor_searcher(_kernel_radius);
}

void Sph_System_Data3::build_neighbor_lists()
{
	Particle_System_Data3::build_neighbor_lists(_kernel_radius);
}

void Sph_System_Data3::compute_mass()
{
	std::vector<Vector3D> points;
	BccLatticePointGenerator pointsGenerator;
	BoundingBox3D sampleBound(
		Vector3D(-1.5 * _kernel_radius, -1.5 * _kernel_radius,
			-1.5 * _kernel_radius),
		Vector3D(1.5 * _kernel_radius, 1.5 * _kernel_radius,
			1.5 * _kernel_radius));

	pointsGenerator.generate(sampleBound, _target_spacing, &points);

	double max_number_density = 0.0;
	Sph_Std_Kernel3 kernel(_kernel_radius);

	for (size_t i = 0; i < points.size(); ++i) {
		const Vector3D& point = points[i];
		double sum = 0.0;

		for (size_t j = 0; j < points.size(); ++j) {
			const Vector3D& neighbor_point = points[j];
			sum += kernel((neighbor_point-point).norm());
		}

		max_number_density = std::max(max_number_density, sum);
	}

	double new_mass = _target_density / max_number_density;
	Particle_System_Data3::set_particle_mass(new_mass);
}
