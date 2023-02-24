#pragma once
#ifndef VECTOR_FIELD3_H_
#define VECTOR_FIELD3_H_

#include "field3.h"
#include <functional>
#include <memory>
#include <Eigen/Dense>

using Vector3D = Eigen::Vector3d;

class Vector_Field3 : public Field3
{
public:
	Vector_Field3();
	virtual ~Vector_Field3();
	
	virtual Vector3D sample(const Vector3D& x) const = 0;

	virtual double divergence(const Vector3D& x) const;

	virtual Vector3D curl(const Vector3D& x) const;

	// return sampler function object.
	virtual std::function<Vector3D(const Vector3D&)> get_sampler() const;
};

typedef std::shared_ptr<Vector_Field3> Vector_Field3_Ptr;

#endif