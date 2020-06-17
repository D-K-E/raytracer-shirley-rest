#ifndef SPHERE_HPP
#define SPHERE_HPP

//
#include <commons.hpp> // vek3, ray
//
#include <hittable.hpp>
//
#include <onb.hpp>
//

class Sphere : public Hittable {
public:
  Sphere() {}
  Sphere(point3 cen, double r, shared_ptr<Material> m)
      : center(cen), radius(r), mat_ptr(m) {}
  bool hit(const Ray &r, double tmin, double tmax,
           HitRecord &rec) const override;
  bool bounding_box(double t0, double t1, Aabb &output_box) const override;
  double pdf_value(const point3 &o, const vec3 &v) const override;
  vec3 random(const point3 &o) const override;
  void get_uv(const vec3 &out_normal, double &u, double &v) const {
    // get sphere uv
    auto phi = atan2(out_normal.z, out_normal.x);
    auto theta = asin(out_normal.y);
    u = 1 - (phi + PI) / (2 * PI);
    v = (theta + PI / 2) / PI;
  }

public:
  point3 center;
  double radius;
  shared_ptr<Material> mat_ptr;
};

double Sphere::pdf_value(const point3 &o, const vec3 &v) const {
  HitRecord rec;
  if (!this->hit(Ray(o, v), 0.001, INF, rec)) {
    return 0;
  }
  vec3 odiff = center - o;

  double cos_theta_max = sqrt(1 - (radius * radius / dot(odiff, odiff)));
  auto solid_angle = 2 * PI * (1 - cos_theta_max);

  return 1 / solid_angle;
}

vec3 Sphere::random(const point3 &o) const {
  vec3 direction = center - o;
  double distance_squared = dot(direction, direction);
  Onb uvw;
  uvw.build_from_w(direction);
  return uvw.local(random_to_sphere(radius, distance_squared));
}

bool Sphere::bounding_box(double t0, double t1, Aabb &output_box) const {
  output_box = Aabb(center - vec3(radius), center + vec3(radius));
  return true;
}

bool Sphere::hit(const Ray &r, double t_min, double t_max,
                 HitRecord &rec) const {
  vec3 oc = r.origin() - center;
  auto a = dot(r.direction(), r.direction());
  auto half_b = dot(oc, r.direction());
  auto c = dot(oc, oc) - radius * radius;

  auto isHit = half_b * half_b - a * c;

  if (isHit > 0) {
    auto root = sqrt(isHit);
    rec.objtype = "sphere";

    double margin = (-half_b - root) / a;
    if (margin < t_max && margin > t_min) {
      rec.dist = margin;
      rec.point = r.at(rec.dist);
      vec3 outward_normal = (rec.point - center) / radius;
      rec.set_face_normal(r, outward_normal);
      vec3 uv_normal = rec.front_face ? rec.normal : -1 * rec.normal;
      get_uv(uv_normal, rec.u, rec.v);
      rec.mat_ptr = mat_ptr;
      return true;
    }

    margin = (-half_b + root) / a;
    if (margin < t_max && margin > t_min) {
      rec.dist = margin;
      rec.point = r.at(rec.dist);
      vec3 outward_normal = to_unit(rec.point - center);
      rec.set_face_normal(r, outward_normal);
      vec3 uv_normal = rec.front_face ? rec.normal : -1 * rec.normal;
      get_uv(uv_normal, rec.u, rec.v);
      rec.mat_ptr = mat_ptr;
      return true;
    }
  }

  return false;
}

#endif
