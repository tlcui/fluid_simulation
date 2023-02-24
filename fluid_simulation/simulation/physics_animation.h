#pragma once
#ifndef PHYSICS_ANIMATION_H_
#define PHYSICS_ANIMATION_H_

#include "animation.h"

class Physics_Animation : public Animation
{
public:
	Physics_Animation();
	virtual ~Physics_Animation();

    bool is_subtimestep_fixed() const;
    void set_is_subtimestep_fixed(bool is_fixed);

    unsigned int get_number_of_fixed_subtimesteps() const;
    void set_number_of_fixed_subtimesteps(unsigned int number);

    Frame get_current_frame() const;

    //!
    //! \brief      Sets current frame cursor (but do not invoke update()).
    //!
    void set_current_frame(const Frame& frame);

    //!
    //! \brief      Returns current time in seconds.
    //!
    //! This function returns the current time which is calculated by adding
    //! current frame + sub-timesteps it passed.
    //!
    double get_current_time_in_seconds() const;

    //! Advances a single frame.
    void advance_single_frame();

protected:
    //! \brief      Called when a single time-step should be advanced.
    //!
    //! When Animation::update function is called, this class will internally
    //! subdivide a frame into sub-steps if needed. Each sub-step, or time-step,
    //! is then taken to move forward in time. This function is called for each
    //! time-step, and a subclass that inherits PhysicsAnimation class should
    //! implement this function for its own physics model.
    //!
    //! \param[in]  timeIntervalInSeconds The time interval in seconds
    //!
    virtual void on_advance_timestep(double timeIntervalInSeconds) = 0;

    
    //! \brief      Returns the required number of sub-timesteps for given time
    //!             interval.
    //!
    //! The required number of sub-timestep can be different depending on the
    //! physics model behind the implementation. Override this function to
    //! implement own logic for model specific sub-timestepping for given
    //! time interval.
    //!
    //! \param[in]  timeIntervalInSeconds The time interval in seconds.
    //!
    //! \return     The required number of sub-timesteps.
    //!
    virtual unsigned int get_number_of_subtimesteps(double timeIntervalInSeconds) const;

    
    //! \brief      Called at frame 0 to initialize the physics state.
    //!
    //! Inheriting classes can override this function to setup initial condition
    //! for the simulation.
    //!
    virtual void on_initialize();

private:
	Frame _current_frame;
	bool _is_subtimestep_fixed = true;
	unsigned int _number_of_fixed_subtimesteps = 1;
	double _current_time = 0.0;

	void on_update(const Frame& frame) final;

	void advance_timestep(double timeIntervalInSeconds);

	void initialize();
};
#endif

