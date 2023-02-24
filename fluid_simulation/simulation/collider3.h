#pragma once
#ifndef COLLIDER3_H_
#define COLLIDER3_H_

#include "surface3.h"
#include <functional>

class Collider3
{
public:
    //! \brief Callback function type for update calls.
    //!
    //! This type of callback function will take the collider pointer, current
    //! time, and time interval in seconds.
    //!
    typedef std::function<void(Collider3*, double, double)> On_Begin_Update_Callback;

    Collider3();
    virtual ~Collider3();

    //! Returns the velocity of the collider at given \p point.
    virtual Vector3D get_velocity_at(const Vector3D& point) const = 0;

    double get_friction_coefficient() const;
    void set_friction_coefficient(double new_coefficient);

    const Surface3Ptr& get_surface() const;

    //! Updates the collider state.
    void update(double current_time_in_seconds, double time_interval_in_seconds);

    //! The callback function takes current simulation time in seconds unit. Use
    //! this callback to track any motion or state changes related to this
    //! collider.
    //!
    //! \param[in]  callback The callback function.
    //!
    void set_on_begin_update_callback(const On_Begin_Update_Callback& callback);

    //! Resolves collision for given point.
    //!
    //! \param radius Radius of the colliding point.
    //! \param restitutionCoefficient Defines the restitution effect.
    //! \param position Input and output position of the point.
    //! \param position Input and output velocity of the point.
    //!
    void resolve_collision(
        double radius,
        double restitutionCoefficient,
        Vector3D* position,
        Vector3D* velocity);

protected:
    //! Internal query result structure.
    struct Collider_Query_Result final {
        double distance;
        Vector3D point;
        Vector3D normal;
        Vector3D velocity;
    };

    void set_surface(const Surface3Ptr& newSurface);

    void get_closest_point(const Surface3Ptr& surface, const Vector3D& query_point, Collider_Query_Result* result) const;

    bool is_penetrating(const Collider_Query_Result& collider_point, const Vector3D& position, double radius);

private:
    Surface3Ptr _surface;
    double _friction_coefficient = 0.;
    On_Begin_Update_Callback _on_update_callback;
};

typedef std::shared_ptr<Collider3> Collider3_Ptr;

#endif 