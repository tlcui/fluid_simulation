#include "particle_system_data3.h"
#include <tbb/parallel_for.h>

static constexpr size_t default_grid_resolution = 64;

Particle_System_Data3::Particle_System_Data3():Particle_System_Data3(0)
{
}

Particle_System_Data3::Particle_System_Data3(size_t numberOfParticles)
{
    _position_index = add_vector_data();
    _velocity_index = add_vector_data();
    _force_index = add_vector_data();

    _neighbor_searcher = std::make_shared<Point_Hash_Grid_Searcher3>
        (default_grid_resolution, default_grid_resolution, default_grid_resolution, 2.0 * _radius);

    resize(numberOfParticles);
}

Particle_System_Data3::Particle_System_Data3(const Particle_System_Data3& other)
{
    set(other);
}

Particle_System_Data3::~Particle_System_Data3()
{

}

void Particle_System_Data3::resize(size_t newNumberOfParticles)
{
    _number_of_particles = newNumberOfParticles;

    for (auto& attr : _scalar_data_list) {
        attr.resize(newNumberOfParticles, 0.0);
    }

    for (auto& attr : _vector_data_list) {
        attr.resize(newNumberOfParticles, Vector3D::Zero());
    }

    _position_float.resize(newNumberOfParticles * 3, 0.0);
}

void Particle_System_Data3::set(const Particle_System_Data3& other)
{
    _radius = other._radius;
    _mass = other._mass;
    _position_index = other._position_index;
    _velocity_index = other._velocity_index;
    _force_index = other._force_index;
    _number_of_particles = other._number_of_particles;

    for (auto& attr : other._scalar_data_list) {
        _scalar_data_list.emplace_back(attr);
    }

    for (auto& attr : other._vector_data_list) {
        _vector_data_list.emplace_back(attr);
    }

    _neighbor_searcher = other._neighbor_searcher->clone();
    _neighbor_lists = other._neighbor_lists;
}

Particle_System_Data3& Particle_System_Data3::operator=(const Particle_System_Data3& other)
{
    set(other);
    return *this;
}

size_t Particle_System_Data3::get_number_of_particles() const
{
    return _number_of_particles;
}

size_t Particle_System_Data3::add_scalar_data(double initialVal)
{
    size_t attrIdx = _scalar_data_list.size();
    _scalar_data_list.emplace_back(_number_of_particles, initialVal); // it is the same as .emplace_back(ScalarData(_number_of_particles, initialVal))
    return attrIdx;
}

size_t Particle_System_Data3::add_vector_data(const Vector3D& initialVal)
{
    size_t attrIdx = _vector_data_list.size();
    _vector_data_list.emplace_back(_number_of_particles, initialVal);
    return attrIdx;
}

double Particle_System_Data3::get_particle_radius() const
{
    return _radius;
}

void Particle_System_Data3::set_particle_radius(double newRadius)
{
    _radius = std::max(newRadius, 0.0);

}

double Particle_System_Data3::get_particle_mass() const
{
    return _mass;
}

void Particle_System_Data3::set_particle_mass(double new_mass)
{
    _mass = std::max(new_mass, 0.0);
}

const std::vector<Vector3D>& Particle_System_Data3::get_positions() const
{
    return _vector_data_list[_position_index];
}

std::vector<Vector3D>& Particle_System_Data3::get_positions()
{
    return _vector_data_list[_position_index];
}

const std::vector<Vector3D>& Particle_System_Data3::get_velocities() const
{
    return _vector_data_list[_velocity_index];
}

std::vector<Vector3D>& Particle_System_Data3::get_velocities()
{
    return _vector_data_list[_velocity_index];
}

const std::vector<Vector3D>& Particle_System_Data3::get_forces() const
{
    return _vector_data_list[_force_index];
}

std::vector<Vector3D>& Particle_System_Data3::get_forces()
{
    return _vector_data_list[_force_index];
}

const std::vector<float>& Particle_System_Data3::get_positions_float() const
{
    return _position_float;
}

std::vector<float>& Particle_System_Data3::get_positions_float()
{
    return _position_float;
}

const std::vector<double>& Particle_System_Data3::scalar_data_at(size_t idx) const
{
    return _scalar_data_list[idx];
}

std::vector<double>& Particle_System_Data3::scalar_data_at(size_t idx)
{
    return _scalar_data_list[idx];
}

const std::vector<Vector3D>& Particle_System_Data3::vector_data_at(size_t idx) const
{
    return _vector_data_list[idx];
}

std::vector<Vector3D>& Particle_System_Data3::vector_data_at(size_t idx)
{
    return _vector_data_list[idx];
}

void Particle_System_Data3::add_particle(const Vector3D& newPosition, const Vector3D& newVelocity, const Vector3D& newForce)
{
    std::vector<Vector3D> new_positions = { newPosition };
    std::vector<Vector3D> new_velocities = { newVelocity };
    std::vector<Vector3D> new_forces = { newForce };

    add_particles(new_positions, new_velocities, new_forces);
}

void Particle_System_Data3::add_particles(const std::vector<Vector3D>& newPositions, const std::vector<Vector3D>& newVelocities, const std::vector<Vector3D>& newForces)
{
    size_t old_number_of_particles = _number_of_particles;
    size_t new_number_of_particles = old_number_of_particles + newPositions.size();

    resize(new_number_of_particles);

    auto& pos = get_positions();
    auto& vel = get_velocities();
    auto& force = get_forces();

    tbb::parallel_for(tbb::blocked_range<size_t>(0, newPositions.size()), [&](const tbb::blocked_range<size_t>& r)
        {
            for (size_t i = r.begin(); i != r.end(); ++i)
            {
                pos[i + old_number_of_particles] = newPositions[i];
            }
        }
    );

    if (newVelocities.size() > 0)
    {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, newVelocities.size()), [&](const tbb::blocked_range<size_t>& r)
            {
                for (size_t i = r.begin(); i != r.end(); ++i)
                {
                    vel[i + old_number_of_particles] = newVelocities[i];
                }
            }
        );
    }

    if (newForces.size() > 0)
    {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, newForces.size()), [&](const tbb::blocked_range<size_t>& r)
            {
                for (size_t i = r.begin(); i != r.end(); ++i)
                {
                    force[i + old_number_of_particles] = newForces[i];
                }
            }
        );
    }
    // _number_of_particles has already been updated in resize()
}

const Point_Neighbor_Searcher3_Ptr& Particle_System_Data3::get_neighbor_searcher() const
{
    return _neighbor_searcher;
}

void Particle_System_Data3::set_neighbor_searcher(const Point_Neighbor_Searcher3_Ptr& new_ptr)
{
    _neighbor_searcher = new_ptr;
}

const std::vector<std::vector<size_t>>& Particle_System_Data3::get_neighbor_lists() const
{
    return _neighbor_lists;
}

void Particle_System_Data3::build_neighbor_searcher(double max_search_radius)
{
    // use hash grid searcher by default
    _neighbor_searcher = std::make_shared<Point_Hash_Grid_Searcher3>
        (default_grid_resolution, default_grid_resolution, default_grid_resolution, 2.0 * max_search_radius);

    _neighbor_searcher->build(get_positions());
}

void Particle_System_Data3::build_neighbor_lists(double max_search_radius)
{
    size_t n = get_number_of_particles();
    _neighbor_lists.resize(n);

    auto& points = get_positions();
    for (size_t i = 0; i < n; ++i)
    {
        Vector3D origin = points[i];
        _neighbor_lists[i].clear();

        _neighbor_searcher->for_each_nearby_point(origin, max_search_radius,
            [&](size_t j, const Vector3D&)
            {
                if(i != j) _neighbor_lists[i].push_back(j);
            });
    }
}
