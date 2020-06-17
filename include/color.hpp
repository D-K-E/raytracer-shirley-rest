#ifndef COLOR_HPP
#define COLOR_HPP

#include <commons.hpp>
#include <iostream>

int cast_color(double d) {
  // renkleri double'dan int tipine donustur
  return static_cast<int>(255.999 * d);
}
int cast_color(double d, bool isClamped) {
  // renkleri double'dan int tipine donustur
  return static_cast<int>(256 * clamp(d, 0.0, 0.999));
}

void write_color(std::ostream &out, color pcolor) {
  out << cast_color(pcolor.x) << ' ' << cast_color(pcolor.y) << ' '
      << cast_color(pcolor.z) << std::endl;
}
void write_color(std::ostream &out, color pcolor, int samples_per_pixel) {
  // replace nans with zero since no nans are equal to each other
  pcolor.x = (pcolor.x != pcolor.x) ? 0.0 : pcolor.x;
  pcolor.y = (pcolor.y != pcolor.y) ? 0.0 : pcolor.y;
  pcolor.z = (pcolor.z != pcolor.z) ? 0.0 : pcolor.z;
  // scale sample
  pcolor /= samples_per_pixel;
  out << cast_color(pcolor.x, true) << ' ' << cast_color(pcolor.y, true) << ' '
      << cast_color(pcolor.z, true) << std::endl;
}

#endif
