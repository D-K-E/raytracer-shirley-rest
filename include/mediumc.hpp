#ifndef MEDIUMC_HPP
#define MEDIUMC_HPP
#include <commons.hpp>
#include <hittable.hpp>
#include <material.hpp>

class ConstantMedium : public Hittable {
public:
  shared_ptr<Hittable> boundary;
  shared_ptr<Material> phasefn;
  double negative_inv_density;

public:
  ConstantMedium(shared_ptr<Hittable> b, double density, shared_ptr<Texture> a)
      : boundary(b), negative_inv_density(-1.0 / density) {
    phasefn = make_shared<Isotropic>(a);
  }
  bool hit(const Ray &r, double t_min, double t_max,
           HitRecord &rec) const override {
    /*
    If that distance is outside the volume, then there is no “hit”. For a
    constant volume we just need the density C and the boundary. I’ll use
    another hittable for the boundary.
     */
    const bool enableDebug = false;
    const bool debugging = enableDebug && random_double() < 0.00001;

    HitRecord rec1, rec2;

    if (!boundary->hit(r, -INF, INF, rec1)) {
      return false;
    }

    if (!boundary->hit(r, rec1.dist + 0.0001, INF, rec2)) {
      return false;
    }

    if (debugging) {
      std::cerr << "\nt0=" << rec1.dist << ", t1=" << rec2.dist << '\n';
    }

    if (rec1.dist < t_min) {
      rec1.dist = t_min;
    }
    if (rec2.dist > t_max) {
      rec2.dist = t_max;
    }
    if (rec1.dist >= rec2.dist) {
      return false;
    }
    if (rec1.dist < 0) {
      rec1.dist = 0;
    }
    const double ray_length = length(r.direction());
    const auto distance_inside_boundary = (rec2.dist - rec1.dist) * ray_length;
    const auto hit_distance = negative_inv_density * log(random_double());

    if (hit_distance > distance_inside_boundary)
      return false;

    rec.dist = rec1.dist + hit_distance / ray_length;
    rec.point = r.at(rec.dist);

    if (debugging) {
      std::cerr << "hit_distance = " << hit_distance << std::endl;
      std::cerr << "rec.t = " << rec.dist << std::endl;
      std::cerr << "rec.point = " << rec.point << std::endl;
    }

    rec.normal = vec3(1, 0, 0); // arbitrary
    rec.front_face = true;      // also arbitrary
    rec.mat_ptr = phasefn;

    return true;
  }
  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    return boundary->bounding_box(t0, t1, output_box);
  }
};

#endif
