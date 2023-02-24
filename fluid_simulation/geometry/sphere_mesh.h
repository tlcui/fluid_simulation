#pragma once
#ifndef SPHERE_MESH_H_
#define SPHERE_MESH_H_

#include "../common_utils.h"
#include "../constants.h"
#include <tbb/parallel_for.h>
#include <vector>

// mesh for a single sphere(particle)
template<typename T = float>
class Sphere_Mesh
{
public:
	Sphere_Mesh(T radius);
	~Sphere_Mesh();

	// the parameter is the center of the sphere
	void initialize_vertices(const Vector3D& sphere = Vector3D::Zero());

public:
	unsigned int* indices;
	T* vertices;
	T radius;

	int X_SEGMENTS = 25;
	int Y_SEGMENTS = 25;
};

template<typename T>
inline Sphere_Mesh<T>::Sphere_Mesh(T radius)
{
	this->radius = radius;
	vertices = new T[(X_SEGMENTS + 1) * (Y_SEGMENTS + 1) * 6]; // position and normal vector
	indices = new unsigned int[X_SEGMENTS * Y_SEGMENTS * 6];


		//int sphere_index = number * X_SEGMENTS * Y_SEGMENTS * 6;
		//int sphere_index_of_vertices = number * (X_SEGMENTS + 1) * (Y_SEGMENTS + 1);
		tbb::parallel_for(tbb::blocked_range<int>(0, X_SEGMENTS), [&](const tbb::blocked_range<int>& r)
			{
				for (int i = r.begin(); i != r.end(); ++i)
				{
					for (int j = 0; j < Y_SEGMENTS; ++j)
					{
						int square_index = i * Y_SEGMENTS + j;
						int index = square_index * 6;
						indices[index + 0] = i * (Y_SEGMENTS + 1) + j;
						indices[index + 1] = (i + 1) * (Y_SEGMENTS + 1) + j;
						indices[index + 2] = i * (Y_SEGMENTS + 1) + j + 1;
						indices[index + 3] = (i + 1) * (Y_SEGMENTS + 1) + j + 1;
						indices[index + 4] = i * (Y_SEGMENTS + 1) + j + 1;
						indices[index + 5] = (i + 1) * (Y_SEGMENTS + 1) + j;
					}
				}
			}
		);
	
}

template<typename T>
inline Sphere_Mesh<T>::~Sphere_Mesh()
{
	delete[] indices;
	delete[] vertices;
}

template<typename T>
inline void Sphere_Mesh<T>::initialize_vertices(const Vector3D& sphere)
{
	//for (int number = 0; number < Number; ++number)
	//{
		//int sphere_index_of_vertices = number * (X_SEGMENTS + 1) * (Y_SEGMENTS + 1) * 6;
		T center_x = sphere.x();
		T center_y = sphere.y();
		T center_z = sphere.z();
		tbb::parallel_for(tbb::blocked_range<int>(0, X_SEGMENTS + 1), [&](const tbb::blocked_range<int>& r)
			{
				for (int i = r.begin(); i != r.end(); ++i)
				{
					T x_seg = static_cast<T>(i) / static_cast<T>(X_SEGMENTS);
					T cos_fi = std::cos(x_seg * 2 * PI<T>());
					T sin_fi = std::sin(x_seg * 2 * PI<T>());
					for (int j = 0; j < Y_SEGMENTS + 1; ++j)
					{
						T y_seg = static_cast<T>(j) / static_cast<T>(Y_SEGMENTS);
						int index = 6 * (i * (Y_SEGMENTS + 1) + j);
						T sin_theta = std::sin(y_seg * PI<T>());
						T cos_theta = std::cos(y_seg * PI<T>());
						T normal_vec_x = sin_theta * cos_fi;
						T normal_vec_y = cos_theta;
						T normal_vec_z = sin_theta * sin_fi;
						vertices[index + 0] = center_x + radius * normal_vec_x;
						vertices[index + 1] = center_y + radius * normal_vec_y;
						vertices[index + 2] = center_z + radius * normal_vec_z;
						vertices[index + 3] = normal_vec_x;
						vertices[index + 4] = normal_vec_y;
						vertices[index + 5] = normal_vec_z;
					}
				}
			}
		);
	//}
}


#endif