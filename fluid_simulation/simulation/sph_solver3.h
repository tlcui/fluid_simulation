#pragma once
#ifndef SPH_SOLVER3_H_
#define SPH_SOLVER3_H_

#include "particle_system_solver3.h"
#include "sph_system_data3.h"
#include "sph_kernels3.h"

class Sph_Solver3 : public Particle_System_Solver3
{
public:
    Sph_Solver3();
    Sph_Solver3(double target_density, double target_spacing, double relative_kernel_radius = 1.8);

    virtual ~Sph_Solver3();

    double get_eos_exponent() const;
    void set_eos_exponent(double eos_exponent);

    double get_negative_pressure_scale() const;
    void set_negative_pressure_scale(double negative_pressure_scale);

    double get_viscosity_coefficient() const;
    void set_viscosity_coefficient(double viscosity_coefficient);

    double get_pseudo_viscosity_coefficient() const;
    void set_pseudo_viscosity_coefficient(double viscosity_coefficient);

    double get_speed_of_sound() const;
    void set_speed_of_sound(double speed_of_sound);

    double get_timestep_limit_scale() const;
    void set_timestep_limit_scale(double timestep_limit_scale);

    Sph_System_Data3_Ptr get_sph_system_data() const;

protected:
    unsigned int get_number_of_subtimesteps(double timestep_in_seconds) const override;

    void accumulate_forces(double timestep_in_seconds) override;

    void on_timestep_start(double timestep_in_seconds) override;
    void on_timestep_end(double timestep_in_seconds) override;

    virtual void accumulate_non_pressure_forces(double timestep_in_seconds);
    virtual void accumulate_pressure_force(double timestep_in_seconds);
    void compute_pressure();

    //! Accumulates the pressure force to the given \p pressureForces array.
    void accumulate_pressure_force(
        const std::vector<Vector3D>& positions,
        const std::vector<double>& densities,
        const std::vector<double>& pressures,
        std::vector<Vector3D>& pressure_forces);

    void accumulate_viscosity_force();

    void compute_pseudo_viscosity(double timestep_in_seconds);

private:
    //! Exponent component of equation-of-state (or Tait's equation).
    double _eos_exponent = 7.0;

    //! Negative pressure scaling factor.
    //! Zero means clamping. One means do nothing.
    double _negative_pressure_scale = 0.;

    //! Viscosity coefficient.
    double _viscosity_coefficient = 0.01;

    //! Pseudo-viscosity coefficient velocity filtering.
    //! This is a minimum "safety-net" for SPH solver which is quite
    //! sensitive to the parameters.
    double _pseudo_viscosity_coefficient = 10.0;

    //! Speed of sound in medium to determin the stiffness of the system.
    //! Ideally, it should be the actual speed of sound in the fluid, but in
    //! practice, use lower value to trace-off performance and compressibility.
    double _speed_of_sound = 100.0;

    //! Scales the max allowed time-step.
    double _timestep_limit_scale = 1.0;
};

inline double compute_pressure_from_eos(
    double density,
    double targetDensity,
    double eosScale,
    double eosExponent,
    double negativePressureScale);

typedef std::shared_ptr<Sph_Solver3> Sph_Solver3_Ptr;

#endif