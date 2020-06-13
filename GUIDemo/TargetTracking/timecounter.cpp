#include "timecounter.h"

// return the elapsed duration in milliseconds
long int timecounter::timeelapsed() {
  auto pt_now = PTIMER::now();
  long int milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(pt_now - pt_start)
          .count();
  pt_start = pt_now;
  return milliseconds;
}

// return the elapsed duration in microseconds
long long timecounter::micro_timeelapsed() {
  auto pt_now = PTIMER::now();
  long long microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(pt_now - pt_start)
          .count();
  pt_start = pt_now;
  return microseconds;
}

// return the UTC time (ISO)
// TODO: C++20 support utc_time
std::string timecounter::getUTCtime() {
  std::time_t result = std::time(nullptr);
  std::string _utc = std::asctime(std::localtime(&result));
  _utc.pop_back();
  return _utc;
}
