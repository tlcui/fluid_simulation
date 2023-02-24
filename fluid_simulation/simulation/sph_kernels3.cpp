#include "sph_kernels3.h"
#include "../constants.h"

Sph_Std_Kernel3::Sph_Std_Kernel3(): h(0), h2(0), h3(0), h5(0)
{
}

Sph_Std_Kernel3::Sph_Std_Kernel3(double kernelRadius): h(kernelRadius), h2(h), h3(h2 * h), h5(h3 * h2)
{
}

Sph_Std_Kernel3::Sph_Std_Kernel3(const Sph_Std_Kernel3& other): h(other.h), h2(other.h2), h3(other.h3), h5(other.h5)
{
}

double Sph_Std_Kernel3::operator()(double distance) const
{
	if (distance * distance >= h2)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - distance * distance / h2;
		return 315.0 / (64.0 * PI_d * h3) * x * x * x;
	}
}

double Sph_Std_Kernel3::get_first_derivative(double distance) const
{
	if (distance >= h)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - distance * distance / h2;
		return -945.0 / (32.0 * PI_d * h5) * distance * x * x;
	}
}

Vector3D Sph_Std_Kernel3::get_gradient(const Vector3D& point) const
{
	double dist = point.norm();
	if (dist > 0.)
	{
		return get_gradient(dist, point / dist);
	}
	else
	{
		return Vector3D::Zero();
	}
}

Vector3D Sph_Std_Kernel3::get_gradient(double distance, const Vector3D& direction) const
{
	return -get_first_derivative(distance) * direction;
}

double Sph_Std_Kernel3::get_second_derivative(double distance) const
{
	if (distance * distance >= h2) {
		return 0.0;
	}
	else {
		double x = distance * distance / h2;
		return 945.0 / (32.0 * PI_d * h5) * (1 - x) * (5 * x - 1);
	}
}

Sph_Spiky_Kernel3::Sph_Spiky_Kernel3(): h(0), h2(0), h3(0), h4(0), h5(0)
{
}

Sph_Spiky_Kernel3::Sph_Spiky_Kernel3(double kernelRadius): h(kernelRadius), h2(h* h), h3(h2* h), h4(h2* h2), h5(h3* h2)
{
}

Sph_Spiky_Kernel3::Sph_Spiky_Kernel3(const Sph_Spiky_Kernel3& other): h(other.h), h2(other.h2), h3(other.h3), h4(other.h4), h5(other.h5)
{
}

double Sph_Spiky_Kernel3::operator()(double distance) const
{
	if (distance * distance >= h2) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance / h;
		return 15.0 / (PI_d * h3) * x * x * x;
	}
}

double Sph_Spiky_Kernel3::get_first_derivative(double distance) const
{
	if (distance >= h) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance / h;
		return -45.0 / (PI_d * h4) * x * x;
	}
}

Vector3D Sph_Spiky_Kernel3::get_gradient(const Vector3D& point) const
{
	double dist = point.norm();
	if (dist > 0.0) {
		return get_gradient(dist, point / dist);
	}
	else {
		return Vector3D::Zero();
	}
}

Vector3D Sph_Spiky_Kernel3::get_gradient(double distance, const Vector3D& direction) const
{
	return -get_first_derivative(distance) * direction;
}

double Sph_Spiky_Kernel3::get_second_derivative(double distance) const
{
	if (distance >= h) {
		return 0.0;
	}
	else {
		double x = 1.0 - distance / h;
		return 90.0 / (PI_d * h5) * x;
	}
}
