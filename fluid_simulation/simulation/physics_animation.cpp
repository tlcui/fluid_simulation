#include "physics_animation.h"
#include "../constants.h"

Physics_Animation::Physics_Animation()
{
	_current_frame.index = -1;
}

Physics_Animation::~Physics_Animation()
{
}

bool Physics_Animation::is_subtimestep_fixed() const
{
	return _is_subtimestep_fixed;
}

void Physics_Animation::set_is_subtimestep_fixed(bool is_fixed)
{
	_is_subtimestep_fixed = is_fixed;
}

unsigned int Physics_Animation::get_number_of_fixed_subtimesteps() const
{
	return _number_of_fixed_subtimesteps;
}

void Physics_Animation::set_number_of_fixed_subtimesteps(unsigned int number)
{
	_number_of_fixed_subtimesteps = number;
}

Frame Physics_Animation::get_current_frame() const
{
	return _current_frame;
}

void Physics_Animation::set_current_frame(const Frame& frame)
{
	_current_frame = frame;
}

double Physics_Animation::get_current_time_in_seconds() const
{
	return _current_time;
}

void Physics_Animation::advance_single_frame()
{
	Frame f = _current_frame;
	update(++f);
}

unsigned int Physics_Animation::get_number_of_subtimesteps(double timeIntervalInSeconds) const
{
	// Returns number of fixed sub-timesteps by default
	return _number_of_fixed_subtimesteps;
}

void Physics_Animation::on_initialize()
{
	// do nothing
}

void Physics_Animation::on_update(const Frame& frame)
{
	if (frame.index > _current_frame.index)
	{
		if (_current_frame.index < 0)
			initialize();

		int32_t number_of_frames = frame.index - _current_frame.index;

		for (int32_t i = 0; i < number_of_frames; i++)
		{
			advance_timestep(frame.time_interval);
		}

		_current_frame = frame;
		//_current_time has already been updated in advance_timestep(), so we don't need to update it here
	}
}

void Physics_Animation::advance_timestep(double timeIntervalInSeconds)
{
	_current_time = _current_frame.time_in_seconds();

	if (_is_subtimestep_fixed)
	{
		const double actual_time_interval = timeIntervalInSeconds / static_cast<double>(_number_of_fixed_subtimesteps);
		
		for (unsigned int i = 0; i < _number_of_fixed_subtimesteps; i++)
		{
			on_advance_timestep(actual_time_interval);
		}

		_current_time += timeIntervalInSeconds;
	}
	else
	{
		double remaining_time = timeIntervalInSeconds;
		while (remaining_time > EPSILON_d)
		{
			unsigned int numsteps = get_number_of_subtimesteps(remaining_time);
			double actual_time_interval = remaining_time / static_cast<double>(numsteps);
			on_advance_timestep(actual_time_interval);

			remaining_time -= actual_time_interval;
			_current_time += actual_time_interval;
		}
	}
}

void Physics_Animation::initialize()
{
	on_initialize();
}
