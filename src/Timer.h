#ifndef __TIMER_H__
#define __TIMER_H__
#include <vector>
#include <string>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

#include "timeutil.h"
#include "ProcessTimer.h"

class Timer
{
	public:

		Timer();
    void startTime(std::string processName);
		void endTime(std::string processName);
		float getFPS(std::string processName);
		void printFPS();
		void printMilliSecs();
    void reset();

		std::vector<ProcessTimer *> m_timers;
		struct timeval m_last_print_time;
};

#endif