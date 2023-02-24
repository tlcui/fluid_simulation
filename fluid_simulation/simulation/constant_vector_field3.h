#pragma once
#ifndef CONSTANT_VECTOR_FIELD3_H_
#define CONSTANT_VECTOR_FIELD3_H_

#include "vector_field3.h"

class Constant_Vector_Field3 : public Vector_Field3
{
private:
	Vector3D _value;

public:
	explicit Constant_Vector_Field3(const Vector3D& value);
	Vector3D sample(const Vector3D& x) const override;
	std::function<Vector3D(const Vector3D&)> get_sampler() const override;
};

typedef std::shared_ptr<Constant_Vector_Field3> Constant_Vector_Field3_Ptr;

#endif