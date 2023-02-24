#include "particle_emitter3.h"

ParticleEmitter3::ParticleEmitter3() {}

ParticleEmitter3::~ParticleEmitter3() {}

const Particle_System_Data3_Ptr& ParticleEmitter3::target() const {
    return _particles;
}

void ParticleEmitter3::setTarget(const Particle_System_Data3_Ptr& particles) {
    _particles = particles;

    onSetTarget(particles);
}

bool ParticleEmitter3::isEnabled() const { return _isEnabled; }

void ParticleEmitter3::setIsEnabled(bool enabled) { _isEnabled = enabled; }

void ParticleEmitter3::update(double currentTimeInSeconds,
    double timeIntervalInSeconds) {
    if (_onBeginUpdateCallback) {
        _onBeginUpdateCallback(this, currentTimeInSeconds,
            timeIntervalInSeconds);
    }

    onUpdate(currentTimeInSeconds, timeIntervalInSeconds);
}

void ParticleEmitter3::onSetTarget(const Particle_System_Data3_Ptr& particles) {
    
}

void ParticleEmitter3::setOnBeginUpdateCallback(
    const OnBeginUpdateCallback& callback) {
    _onBeginUpdateCallback = callback;
}