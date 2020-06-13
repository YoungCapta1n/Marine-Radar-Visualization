#ifndef TIMECOUNTER_H
#define TIMECOUNTER_H

#include <chrono>
#include <ctime>
#include <string>

class timecounter {
  using PTIMER = std::chrono::steady_clock;

 public:
  timecounter() : pt_start(PTIMER::now()) {}

  // return the elapsed duration in milliseconds
  long int timeelapsed();

  // return the elapsed duration in microseconds
  long long micro_timeelapsed();
  // return the UTC time (ISO)
  // TODO: C++20 support utc_time
  std::string getUTCtime();

  ~timecounter() {}

 private:
  PTIMER::time_point pt_start;

};  // end class timecounter

#endif  // TIMECOUNTER_H
