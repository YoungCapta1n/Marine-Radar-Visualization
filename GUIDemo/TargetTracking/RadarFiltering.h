#ifndef RADARFILTERING_H
#define RADARFILTERING_H

#include <cmath>
#include <tuple>
#include "TargetTrackingData.h"

class RadarFiltering {
 public:
  RadarFiltering(double _sigma_w = 1, double _sigma_v = 1)
      : sigma_w(_sigma_w), sigma_v(_sigma_v) {}
  virtual ~RadarFiltering() = default;

  // common alpha beta filtering
  std::tuple<double, double, double, double> NormalFilter(
      const double previous_x, const double previous_vx, const double meas_x,
      const double previous_y, const double previous_vy, const double meas_y,
      const double sample_time);

  // common alpha beta filtering without measurement
  std::tuple<double, double, double, double> NormalFilter(
      const double previous_x, const double previous_vx,
      const double previous_y, const double previous_vy,
      const double sample_time);

  // Hybrid growing-memory alpha beta filtering
  std::tuple<double, double, double, double> GrowingMemoryFilter(
      const double previous_x, const double previous_vx, const double meas_x,
      const double previous_y, const double previous_vy, const double meas_y,
      const double sample_time);

 private:
  double sigma_w;  // process variance
  double sigma_v;  // noise variance

  // alpha-beta filtering for target tracking
  std::tuple<double, double> AlphaBetaFiltering(const double previous_x,
                                                const double previous_v,
                                                const double meas,
                                                const double sample_time,
                                                const double alpha = 0.5,
                                                const double beta = 0.5);

  // Predict the motion, without measurement
  std::tuple<double, double> Predict(const double previous_x,
                                     const double previous_v,
                                     const double sample_time);

};  // end class RadarFiltering

#endif  // RADARFILTERING_H
