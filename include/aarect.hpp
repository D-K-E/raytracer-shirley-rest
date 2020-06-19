#ifndef AARECT_HPP
#define AARECT_HPP
#include <aabb.hpp>
#include <commons.hpp>
#include <hittable.hpp>
#include <material.hpp>

struct AxisInfo {
  unsigned int aligned1;
  unsigned int aligned2;
  unsigned int notAligned;
};

class AaRect : public Hittable {
protected:
  vec3 axis_normal;
  double a0, a1, // aligned1
      b0, b1;    // aligned2
  AxisInfo ax;

public:
  double k;
  shared_ptr<Material> mat_ptr;
  const char *objtype;

public:
  AaRect() {}
  AaRect(double a_0, double a_1, double b_0, double b_1, double _k,
         shared_ptr<Material> mat, vec3 anormal)
      : a0(a_0), a1(a_1), b0(b_0), b1(b_1), k(_k), mat_ptr(mat),
        axis_normal(anormal) {
    if (anormal.z == 1.0) {
      ax.aligned1 = 0;
      ax.aligned2 = 1;
      ax.notAligned = 2;
    } else if (anormal.x == 1.0) {
      ax.aligned1 = 1;
      ax.aligned2 = 2;
      ax.notAligned = 0;
    } else if (anormal.y == 1.0) {
      ax.aligned1 = 0;
      ax.aligned2 = 2;
      ax.notAligned = 1;
    }
  }
  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override {
    /*
       point of intersection satisfies
       both P = O + D*m Point = Origin + Direction * magnitude
       and
       x0 < x_i < x1 y0 < y_i < y1 y0,y1 and x0,x1 being the limits of
       rectangle
     */
    auto t = (k - r.orig()[ax.notAligned]) / r.dir()[ax.notAligned];
    if (t < t0 || t > t1)
      return false;
    double a = r.orig()[ax.aligned1] + t * r.dir()[ax.aligned1];
    double b = r.orig()[ax.aligned2] + t * r.dir()[ax.aligned2];
    bool c1 = a0 < a and a < a1;
    bool c2 = b0 < b and b < b1;
    if ((c1 and c2) == false) {
      return false;
    }
    rec.u = (a - a0) / (a1 - a0);
    rec.v = (b - b0) / (b1 - b0);
    rec.dist = t;
    vec3 outward_normal = axis_normal;
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mat_ptr;
    rec.objtype = this->objtype;
    rec.point = r.at(t);
    return true;
  }
  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    // The bounding box must have non-zero width in each dimension, so pad the Z
    // dimension a small amount.
    point3 p1, p2;
    // choose points with axis
    if (ax.notAligned == 2) {
      p1 = point3(a0, b0, k - 0.0001);
      p2 = point3(a1, b1, k + 0.0001);
    } else if (ax.notAligned == 1) {
      p1 = point3(a0, k - 0.0001, b0);
      p2 = point3(a1, k + 0.0001, b1);
    } else if (ax.notAligned == 0) {
      p1 = point3(k - 0.0001, a0, b0);
      p2 = point3(k + 0.0001, a1, b1);
    }
    output_box = Aabb(p1, p2);
    return true;
  }
};

class XYRect : public AaRect {
public:
  double x0, x1, y0, y1;
  const char *objtype = "xyrect";

public:
  XYRect() {}
  XYRect(double _x0, double _x1, double _y0, double _y1, double _k,
         shared_ptr<Material> mat)
      : AaRect(_x0, _x1, _y0, _y1, _k, mat, vec3(0, 0, 1)), x0(_x0), x1(_x1),
        y0(_y0), y1(_y1) {}
  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override {
    bool isHit = AaRect::hit(r, t0, t1, rec);
    return isHit;
  }
};
class XZRect : public AaRect {
public:
  double x0, x1, z0, z1;
  const char *objtype = "xzrect";

public:
  XZRect() {}
  XZRect(double _x0, double _x1, double _z0, double _z1, double _k,
         shared_ptr<Material> mat)
      : AaRect(_x0, _x1, _z0, _z1, _k, mat, vec3(0, 1, 0)), x0(_x0), x1(_x1),
        z0(_z0), z1(_z1) {}
  // new in the rest of your life book
  double pdf_value(const point3 &origin, const vec3 &v) const override {
    HitRecord rec;
    if (!hit(Ray(origin, v), 0.001, INF, rec)) { // or this->hit
      return 0;
    }

    auto area = (x1 - x0) * (z1 - z0);
    auto distance_squared = rec.dist * rec.dist * dot(v, v);
    auto cosine = fabs(dot(v, rec.normal) / length(v));

    return distance_squared / (cosine * area);
  }

  vec3 random(const point3 &origin) const override {
    // choose random points
    double aligned_random1 = random_double(x0, x1);
    double aligned_random2 = random_double(z0, z1);
    point3 random_point;
    if (ax.notAligned == 0) {
      // yz rect
      random_point = point3(k, aligned_random1, aligned_random2);
    } else if (ax.notAligned == 1) {
      // xz rect
      random_point = point3(aligned_random1, k, aligned_random2);
    } else if (ax.notAligned == 2) {
      random_point = point3(aligned_random1, aligned_random2, k);
    }

    return random_point - origin;
  }
  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override {
    bool isHit = AaRect::hit(r, t0, t1, rec);
    return isHit;
  }
};
class YZRect : public AaRect {
public:
  double y0, y1, z0, z1;
  const char *objtype = "yzrect";

public:
  YZRect() {}
  YZRect(double _y0, double _y1, double _z0, double _z1, double _k,
         shared_ptr<Material> mat)
      : AaRect(_y0, _y1, _z0, _z1, _k, mat, vec3(1, 0, 0)), y0(_y0), y1(_y1),
        z0(_z0), z1(_z1) {}
  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override {
    bool isHit = AaRect::hit(r, t0, t1, rec);
    return isHit;
  }
};

#endif
