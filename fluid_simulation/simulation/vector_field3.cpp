#include "vector_field3.h"

Vector_Field3::Vector_Field3()
{
}

Vector_Field3::~Vector_Field3()
{
}

double Vector_Field3::divergence(const Vector3D& x) const
{
	return 0.0;
}

Vector3D Vector_Field3::curl(const Vector3D& x) const
{
	return Vector3D::Zero();
}

std::function<Vector3D(const Vector3D&)> Vector_Field3::get_sampler() const
{
	const Vector_Field3* self = this;
	return [self](const Vector3D& x) -> Vector3D { return self->sample(x); };
}
