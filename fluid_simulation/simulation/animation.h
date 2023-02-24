#pragma once
#ifndef ANIMATION_H_
#define ANIMATION_H_

#include <memory>

struct Frame final
{
    //! Frame index.
    int index = 0;

    //! Time interval in seconds between two adjacent frames.
    double time_interval = 1.0 / 60.0;

    //! Constructs Frame instance with 1/60 seconds time interval.
    Frame();

    //! Constructs Frame instance with given time interval.
    Frame(int newIndex, double newTimeIntervalInSeconds);

    //! Returns the elapsed time in seconds.
    double time_in_seconds() const;

    //! Advances single frame.
    void advance();

    //! Advances multiple frames.
    //! \param delta Number of frames to advance.
    void advance(int delta);

    //! Advances single frame (prefix).
    Frame& operator++();

    //! Advances single frame (postfix).
    Frame operator++(int);
};

class Animation
{
public:
    Animation();

    virtual ~Animation();

    //! update animation state by calling on_update()
    void update(const Frame& frame);

protected:
    virtual void on_update(const Frame& frame) = 0;

};

//! Shared pointer for the Animation type.
typedef std::shared_ptr<Animation> Animation_ptr;


#endif