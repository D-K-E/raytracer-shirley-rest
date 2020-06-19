// carpilabilir
#ifndef HITTABLE_HPP
#define HITTABLE_HPP
//
#include <aabb.hpp>
#include <commons.hpp>
//
class Material;

struct HitRecord {
  point3 point;
  vec3 normal;
  shared_ptr<Material> mat_ptr;
  double dist;
  bool front_face;
  inline void set_face_normal(const Ray &r, const vec3 &out_normal) {
    // gelen dikme on yuze mi arka yuze mi
    front_face = dot(r.direction, out_normal) < 0;
    normal = front_face ? out_normal : -1 * out_normal;
  }
  double u, v;
};
/* --------------------- HitRecord utils -----------------*/
inline std::ostream &operator<<(std::ostream &out, const HitRecord &h) {
  // yazim
  return out << "point: " << h.point << " normal: " << h.normal
             << " dist: " << h.dist << " front: " << h.front_face
             << " u: " << h.u << " v: " << h.v << " matptr: " << h.mat_ptr;
}

/* --------------------- HitRecord utils end -----------------*/

class Hittable {
public:
  virtual bool hit(const Ray &r, double dist_min, double dist_max,
                   HitRecord &rec) const = 0;
  virtual bool bounding_box(double t0, double t1, Aabb &output_bbox) const = 0;
  virtual double pdf_value(const point3 &o, const vec3 &v) const { return 0.0; }

  virtual vec3 random(const vec3 &o) const { return vec3(1, 0, 0); }
};

class FlipFace : public Hittable {
public:
  shared_ptr<Hittable> ptr;

public:
  FlipFace(shared_ptr<Hittable> p) : ptr(p){};
  virtual bool hit(const Ray &r_in, double t_min, double t_max,
                   HitRecord &rec) const {
    if (ptr->hit(r_in, t_min, t_max, rec) == false)
      return false;

    rec.front_face = not rec.front_face;
    return true;
  }
  virtual bool bounding_box(double t0, double t1, Aabb &output_box) const {
    return ptr->bounding_box(t0, t1, output_box);
  }
};

class Translate : public Hittable {
public:
  Translate(shared_ptr<Hittable> p, const vec3 &disp)
      : ptr(p), displacement(disp) {}

  virtual bool hit(const Ray &r, double t_min, double t_max,
                   HitRecord &rec) const override;
  virtual bool bounding_box(double t0, double t1,
                            Aabb &output_box) const override;

public:
  shared_ptr<Hittable> ptr;
  vec3 displacement;
};

bool Translate::hit(const Ray &r, double t_min, double t_max,
                    HitRecord &rec) const {
  Ray moved_r(r.orig() - displacement, r.dir(), r.time());
  if (!ptr->hit(moved_r, t_min, t_max, rec))
    return false;

  rec.point += displacement;
  rec.set_face_normal(moved_r, rec.normal);

  return true;
}

bool Translate::bounding_box(double t0, double t1, Aabb &output_box) const {
  if (!ptr->bounding_box(t0, t1, output_box))
    return false;

  output_box =
      Aabb(output_box.min() + displacement, output_box.max() + displacement);

  return true;
}

class RotateY : public Hittable {
public:
  RotateY(shared_ptr<Hittable> p, double angle);

  virtual bool hit(const Ray &r, double t_min, double t_max,
                   HitRecord &rec) const override;
  virtual bool bounding_box(double t0, double t1,
                            Aabb &output_box) const override {
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
  point3 max(INF);

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
  point3 origin = r.orig();
  vec3 direction = r.dir();

  origin[0] = cos_theta * r.orig()[0] - sin_theta * r.orig()[2];
  origin[2] = sin_theta * r.orig()[0] + cos_theta * r.orig()[2];

  direction[0] = cos_theta * r.dir()[0] - sin_theta * r.dir()[2];
  direction[2] = sin_theta * r.dir()[0] + cos_theta * r.dir()[2];

  Ray rotated_r(origin, direction, r.time());

  if (!ptr->hit(rotated_r, t_min, t_max, rec))
    return false;

  point3 p = rec.point;
  vec3 normal = rec.normal;

  p[0] = cos_theta * rec.point[0] + sin_theta * rec.point[2];
  p[2] = -sin_theta * rec.point[0] + cos_theta * rec.point[2];

  normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
  normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];

  rec.point = p;
  rec.set_face_normal(rotated_r, normal);

  return true;
}

#endif
