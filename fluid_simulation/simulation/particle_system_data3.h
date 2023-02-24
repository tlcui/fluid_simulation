#pragma once
#ifndef PARTICLE_SYSTEM_DATA_3_H_
#define PARTICLE_SYSTEM_DATA_3_H_

#include <vector>
#include <Eigen/Dense>
#include "point_hash_grid_searcher3.h"

using Vector3D = Eigen::Vector3d;

class Particle_System_Data3
{
public:
    //! Scalar data chunk.
    typedef std::vector<double> ScalarData;

    //! Vector data chunk.
    typedef std::vector<Vector3D> VectorData;

    //! Default constructor.
    Particle_System_Data3();

    //! Constructs particle system data with given number of particles.
    explicit Particle_System_Data3(size_t numberOfParticles);

    //! Copy constructor.
    Particle_System_Data3(const Particle_System_Data3& other);

    //! Destructor.
    virtual ~Particle_System_Data3();

    void resize(size_t newNumberOfParticles);

    //! Copy from other particle system data.
    void set(const Particle_System_Data3& other);

    //! Copy from other particle system data.
    Particle_System_Data3& operator=(const Particle_System_Data3& other);

    //! Return the number of particles.
    size_t get_number_of_particles() const;

    //! \brief      Adds a scalar data layer and returns its index.
    //!
    //! This function adds a new scalar data layer to the system. It can be used
    //! for adding a scalar attribute, such as temperature, to the particles.
    //!
    //! \params[in] initialVal  Initial value of the new scalar data.
    //!
    size_t add_scalar_data(double initialVal = 0.0);

    //! \brief      Adds a vector data layer and returns its index.
    //!
    //! This function adds a new vector data layer to the system. It can be used
    //! for adding a vector attribute, such as vortex, to the particles.
    //!
    //! \params[in] initialVal  Initial value of the new vector data.
    //!
    size_t add_vector_data(const Vector3D& initialVal = Vector3D::Zero());

    //! Return the radius of the particles.
    double get_particle_radius() const;

    //! Set the radius of the particles.
    virtual void set_particle_radius(double newRadius);

    //! Return the mass of the particles.
    double get_particle_mass() const;

    //! Set the mass of the particles.
    virtual void set_particle_mass(double new_mass);

    const std::vector<Vector3D>& get_positions() const;
    std::vector<Vector3D>& get_positions();
    const std::vector<Vector3D>& get_velocities() const;
    std::vector<Vector3D>& get_velocities();
    const std::vector<Vector3D>& get_forces() const;
    std::vector<Vector3D>& get_forces();
    const std::vector<float>& get_positions_float() const;
    std::vector<float>& get_positions_float();

    //! Returns custom scalar data layer at given index (immutable).
   const std::vector<double>& scalar_data_at(size_t idx) const;

    //! Returns custom scalar data layer at given index (mutable).
    std::vector<double>& scalar_data_at(size_t idx);

    //! Returns custom vector data layer at given index (immutable).
    const std::vector<Vector3D>& vector_data_at(size_t idx) const;

    //! Returns custom vector data layer at given index (mutable).
    std::vector<Vector3D>& vector_data_at(size_t idx);

    //! \brief      Adds a particle to the data structure.
    //!
    //! This function will add a single particle to the data structure. For
    //! custom data layers, zeros will be assigned for new particles.
    //! However, this will invalidate neighbor searcher and neighbor lists. It
    //! is users responsibility to call
    //! Particle_System_Data3::buildNeighborSearcher and
    //! Particle_System_Data3::buildNeighborLists to refresh those data.
    //!
    //! \param[in]  newPosition The new position.
    //! \param[in]  newVelocity The new velocity.
    //! \param[in]  newForce    The new force.
    //!
    void add_particle(
        const Vector3D& newPosition,
        const Vector3D& newVelocity = Vector3D::Zero(),
        const Vector3D& newForce = Vector3D::Zero());

    //!
    //! \brief      Adds particles to the data structure.
    //!
    //! This function will add particles to the data structure. For custom data
    //! layers, zeros will be assigned for new particles. However, this will
    //! invalidate neighbor searcher and neighbor lists. It is users
    //! responsibility to call Particle_System_Data3::buildNeighborSearcher and
    //! Particle_System_Data3::buildNeighborLists to refresh those data.
    //!
    //! \param[in]  newPositions  The new positions.
    //! \param[in]  newVelocities The new velocities.
    //! \param[in]  newForces     The new forces.
    //!
    void add_particles(
        const std::vector<Vector3D>& newPositions,
        const std::vector<Vector3D>& newVelocities
        = std::vector<Vector3D>(),
        const std::vector<Vector3D>& newForces
        = std::vector<Vector3D>());

    const Point_Neighbor_Searcher3_Ptr& get_neighbor_searcher() const;
    void  set_neighbor_searcher(const Point_Neighbor_Searcher3_Ptr& new_ptr);
    const std::vector<std::vector<size_t>>& get_neighbor_lists() const;

    void build_neighbor_searcher(double max_search_radius);
    void build_neighbor_lists(double max_search_radius);
private:
    double _radius = 1e-3;
    double _mass = 1e-3;
    size_t _number_of_particles = 0;
    size_t _position_index;
    size_t _velocity_index;
    size_t _force_index;

    std::vector<float> _position_float; // this is used for opengl rendering

    std::vector<ScalarData> _scalar_data_list;
    std::vector<VectorData> _vector_data_list; // may contain position, velocity, force and etc

    Point_Neighbor_Searcher3_Ptr _neighbor_searcher;
    std::vector<std::vector<size_t>> _neighbor_lists;
};

typedef std::shared_ptr<Particle_System_Data3> Particle_System_Data3_Ptr;

#endif
