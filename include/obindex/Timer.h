/**
 * File: Timer.h
 * Date: April 2015
 * Author: Emilio Garcia-Fidalgo 
 * License: see the LICENSE file.
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <sys/time.h>
#include <ctime>

namespace obindex
{

class Timer
{
	public:
		Timer();
		virtual ~Timer();

		void start();
		void stop();
		double getInterval();
	private:
		struct timeval _start_time;
		struct timeval _stop_time;
};

}

#endif /* TIMER_H_ */
