#ifndef PDF_HPP
#define PDF_HPP
//
#include "vec3.hpp"
#include <commons.hpp>
#include <onb.hpp>
//

class Pdf {
public:
  virtual ~Pdf() {}

  virtual double value(const vec3 &direction) const = 0;
  virtual vec3 generate() const = 0;
};

class CosinePdf : public Pdf {
public:
  CosinePdf(const vec3 &w) { uvw.build_from_w(w); }

  double value(const vec3 &direction) const override {
    auto cosine = dot(to_unit(direction), uvw.w());
    return (cosine <= 0) ? 0 : cosine / PI;
  }

  vec3 generate() const override {
    return uvw.local(random_cosine_direction());
  }

public:
  Onb uvw;
};
class HittablePdf : public Pdf {
public:
  HittablePdf(shared_ptr<Hittable> p, const point3 &origin)
      : ptr(p), o(origin) {}

  double value(const vec3 &direction) const override {
    return ptr->pdf_value(o, direction);
  }

  vec3 generate() const override { return ptr->random(o); }

public:
  point3 o;
  shared_ptr<Hittable> ptr;
};

class MixturePdf : public Pdf {
public:
  MixturePdf(shared_ptr<Pdf> p0, shared_ptr<Pdf> p1) {
    p[0] = p0;
    p[1] = p1;
  }

  double value(const vec3 &direction) const override {
    return 0.5 * p[0]->value(direction) + 0.5 * p[1]->value(direction);
  }

  vec3 generate() const override {
    if (random_double() < 0.5) {
      return p[0]->generate();
    } else {
      return p[1]->generate();
    }
  }

public:
  shared_ptr<Pdf> p[2];
};
#endif
