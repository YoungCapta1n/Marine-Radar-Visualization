#include "math_utils.h"

// restrict heading angle or delta heading to (-PI ~ PI)
// compute the delta heading to find the shortest way to rotate
double Normalizeheadingangle(double _heading) noexcept {
  double a = std::fmod(_heading + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}  // Normalizeheadingangle

// convert marine coordinate to cartesian coordinate
double Degree2Rad(double _degree) noexcept {
  return M_PI * _degree / 180.0;
}  // Degree2Rad

// calculate the angle between two 2d vectors
double VectorAngle_2d(const double x1, const double y1,  // [x1, y1]
                      const double x2, const double y2   // [x2, y2]
) {
  double dot = x1 * x2 + y1 * y2;  // dot product between[x1, y1] and [ x2, y2 ]
  double det = x1 * y2 - y1 * x2;  // determinant
  return std::atan2(det, dot);     // atan2(y, x) or atan2(sin, cos)
}  // VectorAngle_2d
