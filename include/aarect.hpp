#ifndef AARECT_HPP
#define AARECT_HPP
#include <aabb.hpp>
#include <commons.hpp>
#include <hittable.hpp>
#include <material.hpp>

struct AxisInfo {
  double aligned1;
  double aligned2;
  double notAligned;
};
inline std::ostream &operator<<(std::ostream &out, const AxisInfo &a) {
  // yazim
  return out << "aligned1: " << a.aligned1 << " aligned2: " << a.aligned2
             << " not aligned: " << a.notAligned;
}

/*
class AaRect : public Hittable {
public:
  vec3 axis_normal;
  double a0, a1, // aligned1
      b0, b1;    // aligned2
  AxisInfo ax;

public:
  double k;
  shared_ptr<Material> mat_ptr;
  const char *mtype = "Aarect";

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

   //  point of intersection satisfies
   //  both P = O + D*m Point = Origin + Direction * magnitude
   //  and
   //  x0 < x_i < x1 y0 < y_i < y1 y0,y1 and x0,x1 being the limits of
   //  rectangle

    auto t = (k - r.origin()[ax.notAligned]) / r.direction()[ax.notAligned];
    if (t < t0 || t > t1)
      return false;
    double a = r.origin()[ax.aligned1] + t * r.direction()[ax.aligned1];
    double b = r.origin()[ax.aligned2] + t * r.direction()[ax.aligned2];
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
  double pdf_value(const point3 &orig, const vec3 &v) const override {
    HitRecord rec;
    if (this->hit(Ray(orig, v), 0.001, INF, rec) == false) {
      //
      return 0;
    }
    auto area = fabs(a0 - a1) * fabs(b0 - b1);
    double dist_sqr = rec.dist * rec.dist * dot(v, v);
    double cosine = fabs(dot(v, rec.normal) / length(v));
    return dist_sqr / (cosine * area);
  }
  vec3 random(const point3 &orig) const override {
    //
    point3 p1;
    auto arnd = random_double(fmin(a0, a1), fmax(a0, a1));
    auto brnd = random_double(fmin(b0, b1), fmax(b0, b1));
    // choose points with axis
    if (ax.notAligned == 2) {
      // normal vec3(0,0,1)
      p1 = point3(arnd, brnd, k);
    } else if (ax.notAligned == 1) {
      // normal vec3(0,1,0)
      p1 = point3(arnd, k, brnd);
    } else if (ax.notAligned == 0) {
      // normal vec3(1,0,0)
      p1 = point3(k, arnd, brnd);
    }
    return p1 - orig;
  }
};
inline std::ostream &operator<<(std::ostream &out, const AaRect &a) {
  // yazim
  return out << "type: " << a.mtype << " a0: " << a.a0 << " a1: " << a.a1
             << " b0: " << a.b0 << " b1: " << a.b1 << " k: " << a.k
             << " axinfo: " << a.ax;
}

class XYRect : public AaRect {
public:
  double x0, x1, y0, y1;
  const char *mtype = "xyrect";

public:
  XYRect() {}
  XYRect(double _x0, double _x1, double _y0, double _y1, double _k,
         shared_ptr<Material> mat)
      : AaRect(_x0, _x1, _y0, _y1, _k, mat, vec3(0, 0, 1)), x0(_x0), x1(_x1),
        y0(_y0), y1(_y1) {}
};
class XZRect : public AaRect {
public:
  double x0, x1, z0, z1;
  const char *mtype = "xzrect";

public:
  XZRect() {}
  XZRect(double _x0, double _x1, double _z0, double _z1, double _k,
         shared_ptr<Material> mat)
      : AaRect(_x0, _x1, _z0, _z1, _k, mat, vec3(0, 1, 0)), x0(_x0), x1(_x1),
        z0(_z0), z1(_z1) {}

  vec3 random(const point3 &origin) const override {
    auto random_point = point3(random_double(x0, x1), k, random_double(z0, z1));
    return random_point - origin;
  }
};
class YZRect : public AaRect {
public:
  double y0, y1, z0, z1;
  const char *mtype = "yzrect";

public:
  YZRect() {}
  YZRect(double _y0, double _y1, double _z0, double _z1, double _k,
         shared_ptr<Material> mat)
      : AaRect(_y0, _y1, _z0, _z1, _k, mat, vec3(1, 0, 0)), y0(_y0), y1(_y1),
        z0(_z0), z1(_z1) {}
};
*/

class FlipFace : public Hittable {
public:
  FlipFace(shared_ptr<Hittable> p) : ptr(p) {}

  bool hit(const Ray &r, double t_min, double t_max,
           HitRecord &rec) const override {
    if (!ptr->hit(r, t_min, t_max, rec))
      return false;

    rec.front_face = !rec.front_face;
    return true;
  }

  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    return ptr->bounding_box(t0, t1, output_box);
  }

public:
  shared_ptr<Hittable> ptr;
};
class XYRect : public Hittable {
public:
  XYRect() {}

  XYRect(double _x0, double _x1, double _y0, double _y1, double _k,
         shared_ptr<Material> mat)
      : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), mp(mat) {}

  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override;

  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    // The bounding box must have non-zero width in each dimension, so pad the Z
    // dimension a small amount.
    output_box = Aabb(point3(x0, y0, k - 0.0001), point3(x1, y1, k + 0.0001));
    return true;
  }

public:
  shared_ptr<Material> mp;
  double x0, x1, y0, y1, k;
};

class XZRect : public Hittable {
public:
  XZRect() {}

  XZRect(double _x0, double _x1, double _z0, double _z1, double _k,
         shared_ptr<Material> mat)
      : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), mp(mat){};

  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override;

  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    // The bounding box must have non-zero width in each dimension, so pad the Y
    // dimension a small amount.
    output_box = Aabb(point3(x0, k - 0.0001, z0), point3(x1, k + 0.0001, z1));
    return true;
  }

  double pdf_value(const point3 &origin, const vec3 &v) const override {
    HitRecord rec;
    if (!this->hit(Ray(origin, v), 0.001, INF, rec))
      return 0;

    auto area = (x1 - x0) * (z1 - z0);
    auto distance_squared = rec.dist * rec.dist * dot(v, v);
    auto cosine = fabs(dot(v, rec.normal) / glm::length(v));

    return distance_squared / (cosine * area);
  }

  vec3 random(const point3 &origin) const override {
    auto random_point = point3(random_double(x0, x1), k, random_double(z0, z1));
    return random_point - origin;
  }

public:
  shared_ptr<Material> mp;
  double x0, x1, z0, z1, k;
};

class YZRect : public Hittable {
public:
  YZRect() {}

  YZRect(double _y0, double _y1, double _z0, double _z1, double _k,
         shared_ptr<Material> mat)
      : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), mp(mat) {}

  bool hit(const Ray &r, double t0, double t1, HitRecord &rec) const override;

  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    // The bounding box must have non-zero width in each dimension, so pad the X
    // dimension a small amount.
    output_box = Aabb(point3(k - 0.0001, y0, z0), point3(k + 0.0001, y1, z1));
    return true;
  }

public:
  shared_ptr<Material> mp;
  double y0, y1, z0, z1, k;
};

bool XYRect::hit(const Ray &r, double t0, double t1, HitRecord &rec) const {
  auto t = (k - r.origin().z) / r.direction().z;
  if (t < t0 || t > t1)
    return false;

  auto x = r.origin().x + t * r.direction().x;
  auto y = r.origin().y + t * r.direction().y;
  if (x < x0 || x > x1 || y < y0 || y > y1)
    return false;

  rec.u = (x - x0) / (x1 - x0);
  rec.v = (y - y0) / (y1 - y0);
  rec.dist = t;
  auto outward_normal = vec3(0, 0, 1);
  rec.set_face_normal(r, outward_normal);
  rec.mat_ptr = mp;
  rec.point = r.at(t);
  rec.objtype = "xyrect";

  return true;
}

bool XZRect::hit(const Ray &r, double t0, double t1, HitRecord &rec) const {
  auto t = (k - r.origin().y) / r.direction().y;
  if (t < t0 || t > t1)
    return false;

  auto x = r.origin().x + t * r.direction().x;
  auto z = r.origin().z + t * r.direction().z;
  if (x < x0 || x > x1 || z < z0 || z > z1)
    return false;

  rec.u = (x - x0) / (x1 - x0);
  rec.v = (z - z0) / (z1 - z0);
  rec.dist = t;
  auto outward_normal = vec3(0, 1, 0);
  rec.set_face_normal(r, outward_normal);
  rec.mat_ptr = mp;
  rec.point = r.at(t);
  rec.objtype = "xzrect";

  return true;
}

bool YZRect::hit(const Ray &r, double t0, double t1, HitRecord &rec) const {
  auto t = (k - r.origin().x) / r.direction().x;
  if (t < t0 || t > t1)
    return false;

  auto y = r.origin().y + t * r.direction().y;
  auto z = r.origin().z + t * r.direction().z;
  if (y < y0 || y > y1 || z < z0 || z > z1)
    return false;

  rec.u = (y - y0) / (y1 - y0);
  rec.v = (z - z0) / (z1 - z0);
  rec.dist = t;
  auto outward_normal = vec3(1, 0, 0);
  rec.set_face_normal(r, outward_normal);
  rec.mat_ptr = mp;
  rec.point = r.at(t);
  rec.objtype = "yzrect";

  return true;
}

#endif
