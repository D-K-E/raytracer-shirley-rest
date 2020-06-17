#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP
#include <cmath>
#include <constants.hpp>
#include <cstdlib>
#include <functional>
#include <numeric>
#include <random>

inline double degree_to_radian(double degree) {
  //
  return degree * PI / 180;
}

//
//
inline double random_double(double min, double max) {
  // random double number in range [min, max]
  static std::uniform_real_distribution<double> distr(min, max);
  static thread_local std::mt19937 gen;
  static std::function<double()> rand_gen = std::bind(distr, gen);
  return rand_gen();
}

inline double random_double() { return random_double(0, 1); }
inline int random_int() {
  // from so: https://stackoverflow.com/a/21238187
  return static_cast<int>(random_double());
}
inline int random_int(const int &min, const int &max) {
  return static_cast<int>(random_double(min, max));
}

//
inline double clamp(double x, double min, double max) {
  //
  bool xmin = x < min;
  bool xmax = x > max;
  if (xmin) {
    return min;
  }
  if (xmax) {
    return max;
  }
  return x;
}

#endif
