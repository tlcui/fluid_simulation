#pragma once
#ifndef PARTICLE_SYSTEM_SOLVER3_H_
#define PARTICLE_SYSTEM_SOLVER3_H_

#include "particle_system_data3.h"
#include "physics_animation.h"
#include "../constants.h"
#include "vector_field3.h"
#include <tbb/parallel_for.h>
#include "collider3.h"
#include "particle_emitter3.h"

class Particle_System_Solver3 : public Physics_Animation
{
private:
	void timestep_start(double timestep_in_seconds);
	void timestep_end(double timestep_in_seconds);
	void accumulate_external_forces();
	void time_integration(double timestep_in_seconds);
	void update_collider(double timestep_in_seconds);
	void update_emitter(double timestep_in_seconds);


	Particle_System_Data3_Ptr _particle_system_data;
	Particle_System_Data3::VectorData _new_positions;
	Particle_System_Data3::VectorData _new_velocities;
	Vector3D _gravity = Vector3D(0.0, -9.8, 0.0);
	double _drag_coefficient = 1e-4;
	double _restitution_coefficient = 0.02;
	Vector_Field3_Ptr _wind; // an external force

	Collider3_Ptr _collider;
	Particle_Emitter3_Ptr _emitter;
	
protected:
	void on_initialize() override;
	void on_advance_timestep(double timestep_in_seconds) override; //the parameter is actually a subtimestep

	virtual void accumulate_forces(double timestep_in_seconds);
	virtual void on_timestep_start(double timestep_in_seconds); //called when a timestep is about to start
	virtual void on_timestep_end(double timestep_in_seconds);//called after a timestep is over

	void resolve_collision();
	void resolve_collision(std::vector<Vector3D>& new_positions, std::vector<Vector3D>& new_velocities);

	void set_particle_system_data(const Particle_System_Data3_Ptr& new_particles);

public:
	Particle_System_Solver3();
	Particle_System_Solver3(double radius, double mass);

	virtual ~Particle_System_Solver3();

	double get_drag_coefficient() const;
	void set_drag_coefficient(double new_drag_coefficient);
	
	double get_restitution_coefficient() const;
	void set_restitution_coefficient(double new_restitution_coefficient);
	
	const Vector3D& get_gravity() const;
	void set_gravity(const Vector3D& new_gravity);

	const Particle_System_Data3_Ptr& get_particle_system_data() const;

	const Vector_Field3_Ptr& get_wind() const;
	void set_wind(const Vector_Field3_Ptr& new_wind);

	const Collider3_Ptr& get_collider() const;
	void set_collider(const Collider3_Ptr& new_collider);

	const Particle_Emitter3_Ptr& get_emitter() const;
	void set_emitter(const Particle_Emitter3_Ptr& new_emitter);
};


#endif
