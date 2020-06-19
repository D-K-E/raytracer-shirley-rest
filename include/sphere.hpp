// Kure objesi
#ifndef SPHERE_HPP
#define SPHERE_HPP

//
#include <commons.hpp> // vek3, ray
//
#include <hittable.hpp>
//
#include <material.hpp>
//
class Sphere : public Hittable {
public:
  point3 center;
  double radius;
  shared_ptr<Material> mat_ptr;

public:
  Sphere() {}
  Sphere(point3 cent, double r, shared_ptr<Material> mat)
      : center(cent), radius(r), mat_ptr(mat){};
  bool hit(const Ray &r, double dist_min, double dist_max,
           HitRecord &record) const {
    // kureye isin vurdu mu onu test eden fonksiyon
    vec3 origin_to_center = r.origin - center;
    double a = dot(r.direction, r.direction);
    double half_b = dot(origin_to_center, r.direction);
    double c = dot(origin_to_center, origin_to_center) - radius * radius;
    double isHit = half_b * half_b - a * c;
    double margin;
    if (isHit > 0) {
      double root = sqrt(isHit);
      margin = (-1 * half_b - root) / a;
      if (margin < dist_max && margin > dist_min) {
        record.dist = margin;
        record.point = r.at(record.dist);
        vec3 out_normal = (record.point - center) / radius;
        record.set_face_normal(r, out_normal);
        if (record.front_face) {
          // ray is outside of the sphere
          // the normal points from center to outside
          get_sphere_uv(record.normal, record.u, record.v);
        } else {
          // ray is inside of the sphere
          // the normal points from inside to center
          get_sphere_uv(-record.normal, record.u, record.v);
        }
        // get_sphere_uv((record.point - center) / radius, record.u, record.v);
        record.mat_ptr = mat_ptr;
        return true;
      }
      margin = (-1 * half_b + root) / a;
      if (margin < dist_max && margin > dist_min) {
        record.dist = margin;
        record.point = r.at(record.dist);
        vec3 out_normal = (record.point - center) / radius;
        record.set_face_normal(r, out_normal);
        if (record.front_face) {
          // ray is outside of the sphere
          // the normal points from center to outside
          get_sphere_uv(record.normal, record.u, record.v);
        } else {
          // ray is inside of the sphere
          // the normal points from inside to center
          get_sphere_uv(-record.normal, record.u, record.v);
        }
        // get_sphere_uv((record.point - center) / radius, record.u, record.v);
        record.mat_ptr = mat_ptr;
        return true;
      }
    }
    return false;
  }
  bool bounding_box(double t0, double t1, Aabb &output_bbox) const {
    //
    output_bbox = Aabb(center - vec3(radius), center + vec3(radius));
    return true;
  }
};

class MovingSphere : public Hittable {
  /* Sonraki hafta */
public:
  point3 center1, center2;
  double time1, time2, radius;
  shared_ptr<Material> mat_ptr;

public:
  MovingSphere() {}
  MovingSphere(point3 cent1, point3 cent2, double t1, double t2, double rad,
               shared_ptr<Material> mat)
      : center1(cent1), center2(cent2), time1(t1), time2(t2), radius(rad),
        mat_ptr(mat){
            // moving sphere
        };
  point3 center(double time) const {
    return center1 + ((time - time1) / (time2 - time1)) * (center2 - center1);
  }
  bool hit(const Ray &r, double dist_min, double dist_max,
           HitRecord &record) const {
    // kureye vurdu mu
    vec3 origin_to_center = r.origin - center(r.time());
    double a = dot(r.direction, r.direction);
    double half_b = dot(origin_to_center, r.direction);
    double c = dot(origin_to_center, origin_to_center) - radius * radius;
    double isHit = half_b * half_b - a * c;
    double margin;
    if (isHit > 0) {
      double root = sqrt(isHit);
      margin = (-1 * half_b - root) / a;
      if (margin < dist_max && margin > dist_min) {
        record.dist = margin;
        record.point = r.at(record.dist);
        vec3 out_normal = (record.point - center(r.time())) / radius;
        record.set_face_normal(r, out_normal);
        record.mat_ptr = mat_ptr;
        return true;
      }
      margin = (-1 * half_b + root) / a;
      if (margin < dist_max && margin > dist_min) {
        record.dist = margin;
        record.point = r.at(record.dist);
        vec3 out_normal = (record.point - center(r.time())) / radius;
        record.set_face_normal(r, out_normal);
        record.mat_ptr = mat_ptr;
        return true;
      }
    }
    return false;
  }
  bool bounding_box(double t0, double t1, Aabb &output_bbox) const {
    //
    Aabb b1(center(t0) - vec3(radius), center(t0) + vec3(radius));
    Aabb b2(center(t1) - vec3(radius), center(t1) + vec3(radius));
    output_bbox = surrounding_box(b1, b2);
    return true;
  }
};

#endif
