#pragma once
#ifndef SPH_DEMO_H_
#define SPH_DEMO_H_

#include "simulation/pcisph_solver3.h"
#include "geometry/implicit_surface_set3.h"
#include "geometry/cylinder3.h"
#include "geometry/box3.h"
#include "geometry/plane3.h"
#include "simulation/volume_particle_emitter3.h"
#include "simulation/rigid_body_collider3.h"
#include <iostream>
#include "geometry/sphere_mesh.h"
#include "geometry/cylinder_mesh.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "render/camera.h"
#include "render/shader.h"


class Sph_Demo
{
private:
	Sph_Solver3_Ptr _solver;
	double _target_spacing;
	double _fps;
	int _number_of_frames;

	void run_simulation(int number_of_frames, double fps);

	Camera camera;

public:
	Sph_Demo(double target_spacing, int number_of_frames, double fps);
	~Sph_Demo();

	void run();
};

void processInput(GLFWwindow* window);

#endif