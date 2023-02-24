#include "constant_vector_field3.h"

Constant_Vector_Field3::Constant_Vector_Field3(const Vector3D& value) : _value(value)
{
}

Vector3D Constant_Vector_Field3::sample(const Vector3D& x) const
{
	return _value;
}

std::function<Vector3D(const Vector3D&)> Constant_Vector_Field3::get_sampler() const
{
	return [this](const Vector3D& x)->Vector3D {return _value; };
}
