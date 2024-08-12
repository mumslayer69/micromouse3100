#pragma once

namespace mtrn3100 {
class CellOdometry {
public:
  CellOdometry()
    : x(0), y(0), heading(0) {}

  void updatePosition(double distance, double angle_deg) {
    double angle_rad = angle_deg * M_PI / 180.0;
    x += distance * cos(angle_rad);
    y += distance * sin(angle_rad);
    heading = angle_rad;
  }

  double getCurrentTotalDistance() const {
    return sqrt((x * x) + (y * y));
  }

  double getCurrentHeading() const {
    return heading;
  }

private:
  double x;
  double y;
  double heading;
};
}