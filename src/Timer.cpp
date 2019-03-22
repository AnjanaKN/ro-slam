#include "Timer.h"

//------------------------------------------------------------------
Timer::Timer() { 
  gettimeofday(&m_last_print_time, NULL); 
}

//------------------------------------------------------------------
void Timer::startTime(std::string processName) {
  for(unsigned int i = 0; i < m_timers.size(); i++)
  {
    if(m_timers[i]->getName() == processName)
    {
      m_timers[i]->startTime();
      return;
    }
  }
  ProcessTimer * newTimer = new ProcessTimer(processName);
  m_timers.push_back(newTimer);
}

//------------------------------------------------------------------
void Timer::endTime(std::string processName) {

  for(unsigned int i = 0; i < m_timers.size(); i++)
  {
    if(m_timers[i]->getName() == processName)
    {
      m_timers[i]->endTime();
      return;
    }
  }
  ProcessTimer * newTimer = new ProcessTimer(processName);
  m_timers.push_back(newTimer);
}

//------------------------------------------------------------------
float Timer::getFPS(std::string processName) {
  for(unsigned int i = 0; i < m_timers.size(); i++)
  {
    if(m_timers[i]->getName() == processName)
    {
      return m_timers[i]->getFPS();
    }
  }
}

//------------------------------------------------------------------
void Timer::printFPS() {
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  if(TIMEVAL_DIFF_SEC(&curr_time, &m_last_print_time) < 1.0) return;

  fprintf(stderr, "-----------------------------\n");
  fprintf(stderr, "Timer: printFPS()\n");
  for(unsigned int i = 0; i < m_timers.size(); i++)
  {
    fprintf(stderr, "%s : %0.2fHz\n", m_timers[i]->getName().c_str(), m_timers[i]->getFPS());
  }
  fprintf(stderr, "-----------------------------\n");
  m_last_print_time = curr_time;
}

//------------------------------------------------------------------
void Timer::printMilliSecs() {
	struct timeval curr_time;
	gettimeofday(&curr_time, NULL);

	fprintf(stderr,"-----------------------------\n");
	fprintf(stderr,"Timer: printMilliSecs()\n");
	for(unsigned int i = 0; i < m_timers.size(); i++)
	{
		fprintf(stderr,"%s : %0.2fms\n", m_timers[i]->getName().c_str(), 1000.0 / m_timers[i]->getFPS());
	}
	fprintf(stderr,"-----------------------------\n");
	m_last_print_time = curr_time;
}

//------------------------------------------------------------------
void Timer::reset() {
	for(unsigned int i = 0; i < m_timers.size(); i++)
	{
    m_timers[i]->reset();
  }
}