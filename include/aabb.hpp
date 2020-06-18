#ifndef AABB_HPP
#define AABB_HPP

#include <commons.hpp>

class Aabb {
public:
  point3 _min;
  point3 _max;

public:
  Aabb(){};
  Aabb(const point3 &a, const point3 &b) {
    //
    _min = a;
    _max = b;
  };
  point3 min() const { return _min; }
  point3 max() const { return _max; }
  bool hit(const Ray &r_in, double tmin, double tmax) const {
    //
    for (int v = 0; v < 3; v++) {
      double m, r, d, invd;
      invd = 1.0 / r_in.dir()[v];
      double t0 = (min()[v] - r_in.orig()[v]) * invd;
      double t1 = (max()[v] - r_in.orig()[v]) * invd;
      if (invd < 0.0) {
        std::swap(t0, t1);
      }
      double tvmin = t0 > tmin ? t0 : tmin;
      double tvmax = t1 < tmax ? t1 : tmax;
      if (tvmax < tvmin) {
        return false;
      }
    }
    return true;
  }
};
Aabb surrounding_box(Aabb b1, Aabb b2) {
  point3 small(fmin(b1.min().x, b2.min().x), fmin(b1.min().y, b2.min().y),
               fmin(b1.min().z, b2.min().z));
  point3 big(fmax(b1.max().x, b2.max().x), fmax(b1.max().y, b2.max().y),
             fmax(b1.max().z, b2.max().z));
  return Aabb(small, big);
}

#endif
