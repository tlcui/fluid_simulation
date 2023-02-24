#pragma once
#ifndef SPH_KERNELS3_H_
#define SPH_KERNELS3_H_

#include "../common_utils.h"

struct Sph_Std_Kernel3 {
    //! Kernel radius.
    double h;

    //! Square of the kernel radius.
    double h2;

    //! Cubic of the kernel radius.
    double h3;

    //! Fifth-power of the kernel radius.
    double h5;

    //! Constructs a kernel object with zero radius.
    Sph_Std_Kernel3();

    //! Constructs a kernel object with given radius.
    explicit Sph_Std_Kernel3(double kernelRadius);

    //! Copy constructor
    Sph_Std_Kernel3(const Sph_Std_Kernel3& other);

    //! Returns kernel function value at given distance.
    double operator()(double distance) const;

    //! Returns the first derivative at given distance.
    double get_first_derivative(double distance) const;

    //! Returns the gradient at a point.
    Vector3D get_gradient(const Vector3D& point) const;

    //! Returns the gradient at a point defined by distance and direction.
    Vector3D get_gradient(double distance, const Vector3D& direction) const;

    //! Returns the second derivative at given distance.
    double get_second_derivative(double distance) const;
};

struct Sph_Spiky_Kernel3
{
    //! Kernel radius.
    double h;

    //! Square of the kernel radius.
    double h2;

    //! Cubic of the kernel radius.
    double h3;

    double h4;

    //! Fifth-power of the kernel radius.
    double h5;

    //! Constructs a kernel object with zero radius.
    Sph_Spiky_Kernel3();

    //! Constructs a kernel object with given radius.
    explicit Sph_Spiky_Kernel3(double kernelRadius);

    //! Copy constructor
    Sph_Spiky_Kernel3(const Sph_Spiky_Kernel3& other);

    //! Returns kernel function value at given distance.
    double operator()(double distance) const;

    //! Returns the first derivative at given distance.
    double get_first_derivative(double distance) const;

    //! Returns the gradient at a point.
    Vector3D get_gradient(const Vector3D& point) const;

    //! Returns the gradient at a point defined by distance and direction.
    Vector3D get_gradient(double distance, const Vector3D& direction) const;

    //! Returns the second derivative at given distance.
    double get_second_derivative(double distance) const;
};
#endif