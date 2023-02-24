#pragma once
#ifndef CYLINDER_MESH_H_
#define CYLINDER_MESH_H_

#include "../common_utils.h"
#include "../constants.h"
#include <tbb/parallel_for.h>
#include <vector>

template<typename T = float>
class Cylinder_Mesh
{
public:
    //! Center of the cylinder.
    Vector3<T> center = Vector3<T>::Zero();

    //! Radius of the cylinder.
    T radius = static_cast<T>(1.0);

    //! Height of the cylinder.
    T height = static_cast<T>(1.0);

    Cylinder_Mesh(T radius, T height);
    ~Cylinder_Mesh();

    void initialize_vertices();

public:
    unsigned int* indices;
    T* vertices;

    int X_SEGMENTS = 50;
    int Y_SEGMENTS = 30;
};

#endif

template<typename T>
inline Cylinder_Mesh<T>::Cylinder_Mesh(T radius, T height)
{
    this->radius = radius;
    this->height = height;

    vertices = new T[(X_SEGMENTS + 1) * (Y_SEGMENTS + 1) * 6]; // position and normal vector
    indices = new unsigned int[X_SEGMENTS * Y_SEGMENTS * 6];

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
inline Cylinder_Mesh<T>::~Cylinder_Mesh()
{
    delete[] indices;
    delete[] vertices;
}

template<typename T>
inline void Cylinder_Mesh<T>::initialize_vertices()
{
    T height_unit = height / Y_SEGMENTS;
    T width_unit = 2 * PI<T>() * radius / X_SEGMENTS;
    tbb::parallel_for(tbb::blocked_range<int>(0, X_SEGMENTS + 1), [&](const tbb::blocked_range<int>& r)
        {
            for (int i = r.begin(); i != r.end(); ++i)
            {
                for (int j = 0; j < Y_SEGMENTS + 1; ++j)
                {
                    int index = 6 * (i * (Y_SEGMENTS + 1) + j);
                    vertices[index + 0] = radius * std::cos(i * width_unit / radius);
                    vertices[index + 1] = -0.5 * height + j * height_unit;
                    vertices[index + 2] = -radius * std::sin(i * width_unit / radius);
                    Vector3<T> normal(vertices[index + 0], 0, vertices[index + 2]);
                    normal.normalize();
                    vertices[index + 3] = normal.x();
                    vertices[index + 4] = normal.y();
                    vertices[index + 5] = normal.z();
                }
            }
        }
    );
}
