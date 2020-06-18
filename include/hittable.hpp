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

#endif
