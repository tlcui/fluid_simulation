#pragma once
#ifndef PCISPH_SOLVER3_H_
#define PCISPH_SOLVER3_H_

#include "sph_solver3.h"

class Pcisph_Solver3 : public Sph_Solver3
{
public:
	Pcisph_Solver3();
    Pcisph_Solver3(
        double target_density,
        double target_spacing,
        double relative_kernel_radius = 1.8);
    virtual ~Pcisph_Solver3();

    double get_max_density_error_ratio() const;
    void set_max_density_error_ratio(double ratio);

    unsigned int get_max_number_of_iterations() const;
    void set_max_number_of_iterations(unsigned int n);

protected:
    void accumulate_pressure_force(double timestep_in_seconds) override;
    void on_timestep_start(double timestep_in_seconds) override;

private:
    double _max_density_error_ratio = 0.01;
    unsigned int _max_number_of_iterations = 5;

    Particle_System_Data3::VectorData _temp_positions;
    Particle_System_Data3::VectorData _temp_velocities;
    Particle_System_Data3::VectorData _pressure_forces;
    Particle_System_Data3::ScalarData _density_errors;

    double compute_delta(double timestep_in_seconds);
    double compute_beta(double timestep_in_seconds);
};

#endif