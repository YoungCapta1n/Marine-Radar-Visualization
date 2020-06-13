#include "RadarFiltering.h"

// common alpha beta filtering
std::tuple<double, double, double, double> RadarFiltering::NormalFilter(
    const double previous_x, const double previous_vx, const double meas_x,
    const double previous_y, const double previous_vy, const double meas_y,
    const double sample_time) {
  double lambda = sample_time * sample_time * sigma_w / sigma_v;
  double r = 0.25 * (lambda + 4 - std::sqrt(lambda * (lambda + 8)));

  double alpha = 1 - r * r;
  double beta = 4 - 2 * alpha - 4 * r;

  auto [new_x, new_vx] = AlphaBetaFiltering(previous_x, previous_vx, meas_x,
                                            sample_time, alpha, beta);
  auto [new_y, new_vy] = AlphaBetaFiltering(previous_y, previous_vy, meas_y,
                                            sample_time, alpha, beta);

  return {new_x, new_vx, new_y, new_vy};
}  // NormalFilter

// common alpha beta filtering without measurement
std::tuple<double, double, double, double> RadarFiltering::NormalFilter(
    const double previous_x, const double previous_vx, const double previous_y,
    const double previous_vy, const double sample_time) {
  auto [new_x, new_vx] = Predict(previous_x, previous_vx, sample_time);
  auto [new_y, new_vy] = Predict(previous_y, previous_vy, sample_time);
  return {new_x, new_vx, new_y, new_vy};
}  // NormalFilter

// Hybrid growing-memory alpha beta filtering
std::tuple<double, double, double, double> RadarFiltering::GrowingMemoryFilter(
    const double previous_x, const double previous_vx, const double meas_x,
    const double previous_y, const double previous_vy, const double meas_y,
    const double sample_time) {
  static int K = 1;
  if (++K > 100) K = 100;
  double mdivide = (K + 1) * (K + 2);
  double alpha = (4 * K + 2) / mdivide;
  double beta = 6 / (sample_time * mdivide);
  auto [new_x, new_vx] = AlphaBetaFiltering(previous_x, previous_vx, meas_x,
                                            sample_time, alpha, beta);
  auto [new_y, new_vy] = AlphaBetaFiltering(previous_y, previous_vy, meas_y,
                                            sample_time, alpha, beta);

  return {new_x, new_vx, new_y, new_vy};
}  // GrowingMemoryFilter

// alpha-beta filtering for target tracking
std::tuple<double, double> RadarFiltering::AlphaBetaFiltering(
    const double previous_x, const double previous_v, const double meas,
    const double sample_time, const double alpha, const double beta) {
  double x_bar = previous_x + sample_time * previous_v;
  double rk = meas - x_bar;
  double new_x = x_bar + rk * alpha;
  double new_v = previous_v + rk * beta / sample_time;

  return {new_x, new_v};
}  // AlphaBetaFiltering

// Predict the motion, without measurement
std::tuple<double, double> RadarFiltering::Predict(const double previous_x,
                                                   const double previous_v,
                                                   const double sample_time) {
  double new_x = previous_x + sample_time * previous_v;
  double new_v = previous_v;

  return {new_x, new_v};
}  // Predict
