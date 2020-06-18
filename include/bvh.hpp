#ifndef BVH_HPP
#define BVH_HPP
//
#include <algorithm>
//
#include <custom/sonraki/commons.hpp>
#include <custom/sonraki/hittable.hpp>
#include <custom/sonraki/hittables.hpp>

class BvhNode : public Hittable {
public:
  shared_ptr<Hittable> left;  // obje
  shared_ptr<Hittable> right; // obje
  Aabb box;

public:
  BvhNode(){};
  BvhNode(std::vector<shared_ptr<Hittable>> &objects, size_t start, size_t end,
          double time1, double time2);
  BvhNode(HittableList &hlist, double time1, double time2)
      : BvhNode(hlist.objects, 0, hlist.size(), time1, time2);
  bool hit(const Ray &r_in, double tmin, double tmax, HitRecord &record) const {
    // objeleri vurdu mu
    if (box.hit(r_in, tmin, tmax) == false) {
      return false;
    }
    bool hleft = left->hit(r_in, tmin, tmax, record);
    double ntmax = hleft ? record.dist : tmax;
    bool hright = right->hit(r_in, tmin, ntmax, record);
    return hleft || hright;
  }
  bool bounding_box(double t1, double t2, Aabb &output_bbox) const {
    output_bbox = box;
    return true;
  };
};

bool box_compare(const shared_ptr<Hittable> a, const shared_ptr<Hittable> b,
                 int ax) {
  // kutulari eksenlerini kullanarak kontrol et
  Aabb ba;
  Aabb bb;
  if ((a->bounding_box(0, 0, ba) == false) ||
      (b->bounding_box(0, 0, bb) == false)) {
    //
    std::cerr << "No bounding box in BVH node" << std::endl;
  }
  return ba.min()[ax] < bb.min()[ax];
}
bool box_compare_x(const shared_ptr<Hittable> a, const shared_ptr<Hittable> b) {
  //
  return box_compare(a, b, 0);
}
bool box_compare_y(const shared_ptr<Hittable> a, const shared_ptr<Hittable> b) {
  //
  return box_compare(a, b, 1);
}
bool box_compare_z(const shared_ptr<Hittable> a, const shared_ptr<Hittable> b) {
  //
  return box_compare(a, b, 2);
}
BvhNode::BvhNode(std::vector<shared_ptr<Hittable>> &objects, size_t start,
                 size_t end, double time1, double time2) {
  //
  int ax = random_int(0, 2);
  typedef bool comparator(const shared_ptr<Hittable>,
                          const shared_ptr<Hittable>);
  if (ax == 0) {
    comparator = box_compare_x;
  } else if (ax == 1) {
    comparator = box_compare_y;
  } else if (ax == 2) {
    comparator = box_compare_z;
  }
  size_t object_span = end - start;
  if (object_span == 1) {
    left = right = objects[start];
  } else if (object_span == 2) {
    if (comparator(objects[start], objects[start + 1])) {
      left = objects[start];
      right = objects[start + 1];
    } else {
      left = objects[start + 1];
      right = objects[start];
    }
  } else {
    std::sort(objects.begin() + start, objects.begin() + end, comparator);
    double mid = start + object_span / 2;
    left = make_shared<BvhNode>(objects, start, mid, time1, time2);
    right = make_shared<BvhNode>(objects, mid, end, time1, time2);
  }
  Aabb bleft;
  Aabb bright;
  if ((left->bounding_box(time1, time2, bleft) == false) ||
      (right->bounding_box(time1, time2, bright) == false)) {
    std::cerr << "" << std::endl;
  }
  box = surrounding_box(bleft, bright);
}

#endif
