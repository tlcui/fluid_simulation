#pragma once
#ifndef SPH_SYSTEM_DATA3_
#define SPH_SYSTEM_DATA3_

#include "particle_system_data3.h"
#include <tbb/parallel_for.h>

class Sph_System_Data3 : public Particle_System_Data3
{
public:
	Sph_System_Data3();
	explicit Sph_System_Data3(size_t number_of_particles);
	Sph_System_Data3(const Sph_System_Data3& other);
	virtual ~Sph_System_Data3();

	void set(const Sph_System_Data3& other);
	Sph_System_Data3& operator=(const Sph_System_Data3& other);

	double get_target_spacing() const;

	//! Sets the target particle spacing in meters.
	//!
	//! Once this function is called, hash grid and density should be
	//! updated using updateHashGrid() and updateDensities).
	//!
	void set_target_spacing(double spacing);

	//! Sets the radius.
	//!
	//! Sets the radius of the particle system. The radius will be interpreted
	//! as target spacing.
	//!
	void set_particle_radius(double new_radius) override;

	//! Sets the relative kernel radius.
	//!
	//! Sets the relative kernel radius compared to the target particle
	//! spacing (i.e. kernel radius / target spacing).
	//! Once this function is called, hash grid and density should
	//! be updated using updateHashGrid() and updateDensities).
	//!
	void set_relative_kernel_radius(double relativeRadius);
	double get_relative_kernel_radius() const;

	void set_particle_mass(double new_mass) override;

	const std::vector<double>& get_densities() const;
	std::vector<double>& get_densities();

	double get_target_density() const;

	void set_target_density(double target_density);

	void update_densities();

	const std::vector<double>& get_pressures() const;
	std::vector<double>& get_pressures();

	double get_kernel_radius() const;
	void set_kernel_radius(double kernel_radius);

	double get_sum_of_kernel_nearby(const Vector3D& origin) const;

	//! Returns interpolated value at given origin point.
	double interpolate(const Vector3D& origin, const std::vector<double>& values) const;
	Vector3D interpolate(const Vector3D& origin, const std::vector<Vector3D>& values) const;

	//! Returns the gradient of the given values at i-th particle
	Vector3D get_gradient_at(size_t i, const std::vector<double>& values) const;

	//! Returns the laplacian of the given values at i-th particle.
	double get_laplacian_at(size_t i, const std::vector<double>& values) const;
	Vector3D get_laplacian_at(size_t i, const std::vector<Vector3D>& values) const;

	//! Builds neighbor searcher with kernel radius.
	void build_neighbor_searcher();

	//! Builds neighbor lists with kernel radius.
	void build_neighbor_lists();



private:
	//target density in kg/m^3
	double _target_density;

	//target spacing in meters
	double _target_spacing = 0.1;

	//this is the h in W(r,h)
	double _kernel_radius;

	//! Relative radius of SPH kernel.
	//! SPH kernel radius divided by target spacing.
	double _kernel_radius_over_target_spacing = 1.8;

	size_t _pressure_index;
	size_t _density_index;

	//compute mass based on target density and spacing
	void compute_mass();
};

typedef std::shared_ptr<Sph_System_Data3> Sph_System_Data3_Ptr;

#endif