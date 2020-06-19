#ifndef HITTABLES_HPP
#define HITTABLES_HPP

//
#include <commons.hpp>
//
#include <hittable.hpp>
#include <memory>
#include <vector>
//

class HittableList : public Hittable {
public:
  std::vector<std::shared_ptr<Hittable>> objects;
  HittableList() {}
  HittableList(shared_ptr<Hittable> object) { add(object); }
  void add(shared_ptr<Hittable> object) { objects.push_back(object); }
  void clear() { objects.clear(); }
  bool hit(const Ray &r, double dist_min, double dist_max,
           HitRecord &record) const override {
    // isin herhangi bir objeye vurdu mu vurmadi mi onu kontrol eden
    // fonksiyon
    HitRecord temp;
    bool hit_ = false;
    double current_closest = dist_max;
    for (const shared_ptr<Hittable> &object : objects) {
      // int i=0; i < objects.size(); i++
      if (object->hit(r, dist_min, dist_max, temp)) {
        hit_ = true;
        current_closest = temp.dist;
        record = temp;
      }
    }
    return hit_;
  }
  bool bounding_box(double t0, double t1, Aabb &output_box) const override {
    if (objects.empty())
      return false;

    Aabb temp_box;
    bool first_true = objects[0]->bounding_box(t0, t1, temp_box);

    if (!first_true)
      return false;

    output_box = temp_box;

    for (const auto &object : objects) {
      if (!object->bounding_box(t0, t1, temp_box)) {
        return false;
      }
      output_box = surrounding_box(output_box, temp_box);
    }

    return true;
  }

  double pdf_value(const point3 &o, const vec3 &v) const override {
    auto weight = 1.0 / objects.size();
    double sum = 0.0;

    for (const auto &object : objects) {
      sum += weight * object->pdf_value(o, v);
    }

    return sum;
  }
  vec3 random(const vec3 &o) const override {
    auto int_size = static_cast<int>(objects.size());
    return objects[random_int(0, int_size - 1)]->random(o);
  }
  //
};

#endif
