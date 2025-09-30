#ifndef _POLYNOMIAL_TRAJ_H_
#define _POLYNOMIAL_TRAJ_H_

#include <Eigen/Eigen>
#include <vector>
#include <cmath>

class PolynomialTraj {
public:
  PolynomialTraj() {}

  void generateFromWaypoints(const std::vector<Eigen::Vector3d>& waypoints, double segment_time) {
    int N = waypoints.size();
    if (N < 2) return;

    times_.clear();
    total_time_ = (N - 1) * segment_time;

    for (int i = 0; i < 3; i++) coefs_[i].clear();

    for (int i = 0; i < N - 1; i++) {
      Eigen::Vector3d p0 = waypoints[i];
      Eigen::Vector3d p1 = waypoints[i + 1];

      for (int d = 0; d < 3; d++) {
        std::vector<double> coef(6, 0.0);
        double t = segment_time;

        coef[0] = p0(d);
        coef[1] = 0;
        coef[2] = 0;
        coef[3] = 10 * (p1(d) - p0(d)) / pow(t, 3);
        coef[4] = -15 * (p1(d) - p0(d)) / pow(t, 4);
        coef[5] = 6 * (p1(d) - p0(d)) / pow(t, 5);

        coefs_[d].push_back(coef);
      }
      times_.push_back(segment_time);
    }
  }

  Eigen::Vector3d evaluate(double t) const {
    if (t < 0) t = 0;
    if (t > total_time_) t = total_time_;

    int seg = std::min((int)(t / times_[0]), (int)times_.size() - 1);
    double tau = t - seg * times_[0];

    Eigen::Vector3d pos;
    for (int d = 0; d < 3; d++) {
      const std::vector<double>& c = coefs_[d][seg];
      pos(d) = c[0] + c[1]*tau + c[2]*pow(tau,2) + c[3]*pow(tau,3) + c[4]*pow(tau,4) + c[5]*pow(tau,5);
    }
    return pos;
  }

  double getTimeSum() const { return total_time_; }
  std::vector<double> getTimes() const { return times_; }
  std::vector<std::vector<double>> getCoef(int dim) const { return coefs_[dim]; }

private:
  std::vector<std::vector<double>> coefs_[3]; // [x,y,z]
  std::vector<double> times_;
  double total_time_;
};

#endif

