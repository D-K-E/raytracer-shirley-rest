// carpilabilir
#ifndef HITTABLE_HPP
#define HITTABLE_HPP
//
#include "constants.hpp"
#include <aabb.hpp>
#include <cmath>
#include <commons.hpp>
//
//

class Material;

struct HitRecord {
  point3 point;
  vec3 normal;
  shared_ptr<Material> mat_ptr;
  double dist;
  double u;
  double v;
  bool front_face;
  const char *objtype;

  inline void set_face_normal(const Ray &r, const vec3 &outward_normal) {
    front_face = dot(r.direction(), outward_normal) < 0;
    normal = front_face ? outward_normal : -1 * outward_normal;
  }
};

class Hittable {
public:
  virtual bool hit(const Ray &r, double t_min, double t_max,
                   HitRecord &rec) const = 0;
  virtual bool bounding_box(double t0, double t1, Aabb &output_box) const = 0;

  virtual double pdf_value(const vec3 &o, const vec3 &v) const { return 0.0; }

  virtual vec3 random(const vec3 &o) const { return vec3(1, 0, 0); }
};

class Translate : public Hittable {
public:
  Translate(shared_ptr<Hittable> p, const vec3 &displacement)
      : ptr(p), offset(displacement) {}

  bool hit(const Ray &r, double t_min, double t_max,
           HitRecord &rec) const override {
    Ray moved_r(r.origin() - offset, r.direction(), r.time());
    if (!ptr->hit(moved_r, t_min, t_max, rec))
      return false;

    rec.point += offset;
    rec.set_face_normal(moved_r, rec.normal);

    return true;
  }

  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    if (!ptr->bounding_box(t0, t1, output_box))
      return false;

    output_box = Aabb(output_box.min() + offset, output_box.max() + offset);

    return true;
  }

public:
  shared_ptr<Hittable> ptr;
  vec3 offset;
};

class Rotator : public Hittable {
public:
  vec3 rotation_axis;
  double angle_radian;
  shared_ptr<Hittable> ptr;
  bool hasbox;
  Aabb bbox;

public:
  Rotator(shared_ptr<Hittable> _ptr, double angle_degree, const vec3 &axis)
      : ptr(_ptr), rotation_axis(axis),
        angle_radian(degree_to_radian(angle_degree)) {
    //
    ptr->bounding_box(0, 1, bbox);
    point3 pmin(INF);
    point3 pmax(-INF);
    int arr[2] = {0, 1};
    for (const int i : arr) {
      for (const int j : arr) {
        for (const int k : arr) {

          auto x = i * bbox.max().x + (1 - i) * bbox.min().x;
          auto y = j * bbox.max().y + (1 - j) * bbox.min().y;
          auto z = k * bbox.max().z + (1 - k) * bbox.min().z;
          vec3 npos = rotate(vec3(x, y, z));

          int fs[] = {0, 1, 2};

          for (const int f : fs) {
            //
            pmin[f] = fmin(pmin[f], npos[f]);
            pmax[f] = fmax(pmax[f], npos[f]);
          }
        }
      }
    }
    bbox = Aabb(pmin, pmax);
  };

  bool hit(const Ray &r, double tmin, double tmax,
           HitRecord &rec) const override {
    //
    auto origin = r.origin();
    auto dir = r.direction();
    auto norigin = rotate(origin);
    auto ndir = rotate(dir);

    Ray rotated_r(norigin, ndir, r.time());
    if (ptr->hit(rotated_r, tmin, tmax, rec) == false) {
      //
      return false;
    }
    vec3 p = rec.point;
    vec3 n = rec.normal;
    auto rp = rotate(p);
    auto rn = rotate(n);
    rec.point = rp;
    rec.set_face_normal(rotated_r, rn);
    return true;
  }
  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    output_box = bbox;
    return hasbox;
  }
  vec3 rotate(const vec3 &pos) const {
    /*
     rotate position using rodriguez formula:
        p_{rot} = p \cos(\theta) + (k \times p) \sin(\theta) +
                  k (k \cdot p)(1 - \cos(\theta))
     */
    double costheta = cos(angle_radian);
    double sintheta = sin(angle_radian);
    vec3 firstTerm = pos * costheta;
    vec3 secondTerm = cross(rotation_axis, pos) * sintheta;
    vec3 thirdTerm = rotation_axis * dot(rotation_axis, pos) * (1 - costheta);
    return firstTerm + secondTerm + thirdTerm;
  }
};
/*
class RotateY : public Rotator {

public:
  RotateY(shared_ptr<Hittable> _ptr, double angle_degree)
      : Rotator(_ptr, angle_degree, vec3(0, 1, 0)) {}
};
*/
class RotateY : public Hittable {
public:
  RotateY(shared_ptr<Hittable> p, double angle);

  bool hit(const Ray &r, double t_min, double t_max,
           HitRecord &rec) const override;
  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    output_box = bbox;
    return hasbox;
  }

public:
  shared_ptr<Hittable> ptr;
  double sin_theta;
  double cos_theta;
  bool hasbox;
  Aabb bbox;
};

RotateY::RotateY(shared_ptr<Hittable> p, double angle) : ptr(p) {
  auto radians = degree_to_radian(angle);
  sin_theta = sin(radians);
  cos_theta = cos(radians);
  hasbox = ptr->bounding_box(0, 1, bbox);

  point3 min(INF);
  point3 max(-INF);

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        auto x = i * bbox.max().x + (1 - i) * bbox.min().x;
        auto y = j * bbox.max().y + (1 - j) * bbox.min().y;
        auto z = k * bbox.max().z + (1 - k) * bbox.min().z;

        auto newx = cos_theta * x + sin_theta * z;
        auto newz = -sin_theta * x + cos_theta * z;

        vec3 tester(newx, y, newz);

        for (int c = 0; c < 3; c++) {
          min[c] = fmin(min[c], tester[c]);
          max[c] = fmax(max[c], tester[c]);
        }
      }
    }
  }

  bbox = Aabb(min, max);
}

bool RotateY::hit(const Ray &r, double t_min, double t_max,
                  HitRecord &rec) const {
  auto origin = r.origin();
  auto direction = r.direction();

  origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
  origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];

  direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
  direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

  Ray rotated_r(origin, direction, r.time());

  if (!ptr->hit(rotated_r, t_min, t_max, rec))
    return false;

  auto p = rec.point;
  auto normal = rec.normal;

  p[0] = cos_theta * rec.point[0] + sin_theta * rec.point[2];
  p[2] = -sin_theta * rec.point[0] + cos_theta * rec.point[2];

  normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
  normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];

  rec.point = p;
  rec.set_face_normal(rotated_r, normal);

  return true;
}

class RotateX : public Rotator {

public:
  RotateX(shared_ptr<Hittable> _ptr, double angle_degree)
      : Rotator(_ptr, angle_degree, vec3(1, 0, 0)) {}
};
class RotateZ : public Rotator {

public:
  RotateZ(shared_ptr<Hittable> _ptr, double angle_degree)
      : Rotator(_ptr, angle_degree, vec3(0, 0, 1)) {}
};

#endif
