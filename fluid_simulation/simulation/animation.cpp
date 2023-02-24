#include "animation.h"

Frame::Frame()
{
}

Frame::Frame(int newIndex, double newTimeIntervalInSeconds): index(newIndex), time_interval(newTimeIntervalInSeconds)
{
}

double Frame::time_in_seconds() const
{
	return index * time_interval;
}

void Frame::advance()
{
	index++;
}

void Frame::advance(int delta)
{
	index += delta;
}

Frame& Frame::operator++()
{
	index++;
	return *this;
}

Frame Frame::operator++(int)
{
	Frame result = *this;
	index++;
	return result;
}

Animation::Animation(){}
Animation::~Animation(){}

void Animation::update(const Frame& frame)
{
	on_update(frame);
}
