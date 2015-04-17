/**
 * File: Timer.cpp
 * Date: April 2015
 * Author: Emilio Garcia-Fidalgo 
 * License: see the LICENSE file.
 */

#include "obindex/Timer.h"

namespace obindex
{

Timer::Timer()
{
	start();
}

Timer::~Timer()
{
}

void Timer::start()
{
	gettimeofday(&_start_time, 0);
	_stop_time = _start_time;
}

void Timer::stop()
{
	gettimeofday(&_stop_time, 0);
}

double Timer::getInterval()
{
	double start = double(_start_time.tv_sec) + double(_start_time.tv_usec) / 1000000.0;
	double stop = double(_stop_time.tv_sec) + double(_stop_time.tv_usec) / 1000000.0;
	return stop - start;
}

}
