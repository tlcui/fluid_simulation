#pragma once
#ifndef VOLUME_PARTICLE_EMITTER3_H_
#define VOLUME_PARTICLE_EMITTER3_H_

#include "particle_emitter3.h"
#include "point_generator3.h"
#include "bounding_box3.h"
#include "implicit_surface3.h"
#include <memory>
#include <random>

//! \brief 3-D volumetric particle emitter.
//!
//! This class emits particles from volumetric geometry.
//!
class VolumeParticleEmitter3 final : public ParticleEmitter3 {
public:

    //!
    //! Constructs an emitter that spawns particles from given implicit surface
    //! which defines the volumetric geometry. Provided bounding box limits
    //! the particle generation region.
    //!
    //! \param[in]  implicitSurface         The implicit surface.
    //! \param[in]  maxRegion               The max region.
    //! \param[in]  spacing                 The spacing between particles.
    //! \param[in]  initialVel              The initial velocity.
    //! \param[in]  linearVel               The linear velocity of the emitter.
    //! \param[in]  angularVel              The angular velocity of the emitter.
    //! \param[in]  maxNumberOfParticles    The max number of particles to be
    //!                                     emitted.
    //! \param[in]  jitter                  The jitter amount between 0 and 1.
    //! \param[in]  isOneShot               True if emitter gets disabled after one shot.
    //! \param[in]  allowOverlapping        True if particles can be overlapped.
    //! \param[in]  seed                    The random seed.
    //!
    VolumeParticleEmitter3(
        const ImplicitSurface3Ptr& implicitSurface,
        const BoundingBox3D& maxRegion,
        double spacing,
        const Vector3D& initialVel = Vector3D::Zero(),
        const Vector3D& linearVel = Vector3D::Zero(),
        const Vector3D& angularVel = Vector3D::Zero(),
        size_t maxNumberOfParticles = std::numeric_limits<size_t>::max(),
        double jitter = 0.0,
        bool isOneShot = true,
        bool allowOverlapping = false,
        uint32_t seed = 0);

    //!
    //! \brief      Sets the point generator.
    //!
    //! This function sets the point generator that defines the pattern of the
    //! point distribution within the volume.
    //!
    //! \param[in]  newPointsGen The new points generator.
    //!
    void setPointGenerator(const PointGenerator3Ptr& newPointsGen);

    //! Returns source surface.
    const ImplicitSurface3Ptr& surface() const;

    //! Sets the source surface.
    void setSurface(const ImplicitSurface3Ptr& newSurface);

    //! Returns max particle gen region.
    const BoundingBox3D& maxRegion() const;

    //! Sets the max particle gen region.
    void setMaxRegion(const BoundingBox3D& newBox);

    //! Returns jitter amount.
    double jitter() const;

    //! Sets jitter amount between 0 and 1.
    void setJitter(double newJitter);

    //! Returns true if particles should be emitted just once.
    bool isOneShot() const;

    //!
    //! \brief      Sets the flag to true if particles are emitted just once.
    //!
    //! If true is set, the emitter will generate particles only once even after
    //! multiple emit calls. If false, it will keep generating particles from
    //! the volumetric geometry. Default value is true.
    //!
    //! \param[in]  newValue True if particles should be emitted just once.
    //!
    void setIsOneShot(bool newValue);

    //! Returns true if particles can be overlapped.
    bool allowOverlapping() const;

    //!
    //! \brief      Sets the flag to true if particles can overlap each other.
    //!
    //! If true is set, the emitter will generate particles even if the new
    //! particles can find existing nearby particles within the particle
    //! spacing.
    //!
    //! \param[in]  newValue True if particles can be overlapped.
    //!
    void setAllowOverlapping(bool newValue);

    //! Returns max number of particles to be emitted.
    size_t maxNumberOfParticles() const;

    //! Sets the max number of particles to be emitted.
    void setMaxNumberOfParticles(size_t newMaxNumberOfParticles);

    //! Returns the spacing between particles.
    double spacing() const;

    //! Sets the spacing between particles.
    void setSpacing(double newSpacing);

    //! Sets the initial velocity of the particles.
    Vector3D initialVelocity() const;

    //! Returns the initial velocity of the particles.
    void setInitialVelocity(const Vector3D& newInitialVel);

    //! Returns the linear velocity of the emitter.
    Vector3D linearVelocity() const;

    //! Sets the linear velocity of the emitter.
    void setLinearVelocity(const Vector3D& newLinearVel);

    //! Returns the angular velocity of the emitter.
    Vector3D angularVelocity() const;

    //! Sets the linear velocity of the emitter.
    void setAngularVelocity(const Vector3D& newAngularVel);

private:
    std::mt19937 _rng;

    ImplicitSurface3Ptr _implicitSurface;
    BoundingBox3D _bounds;
    double _spacing;
    Vector3D _initialVel;
    Vector3D _linearVel;
    Vector3D _angularVel;
    PointGenerator3Ptr _pointsGen;

    size_t _maxNumberOfParticles = std::numeric_limits<size_t>::max();
    size_t _numberOfEmittedParticles = 0;

    double _jitter = 0.0;
    bool _isOneShot = true;
    bool _allowOverlapping = false;

    //!
    //! \brief      Emits particles to the particle system data.
    //!
    //! \param[in]  currentTimeInSeconds    Current simulation time.
    //! \param[in]  timeIntervalInSeconds   The time-step interval.
    //!
    void onUpdate(
        double currentTimeInSeconds,
        double timeIntervalInSeconds) override;

    void emit(
        const Particle_System_Data3_Ptr& particles,
        std::vector<Vector3D>* newPositions,
        std::vector<Vector3D>* newVelocities);

    double random();

    Vector3D velocityAt(const Vector3D& point) const;
};

//! Shared pointer for the VolumeParticleEmitter3 type.
typedef std::shared_ptr<VolumeParticleEmitter3> VolumeParticle_Emitter3_Ptr;

#endif