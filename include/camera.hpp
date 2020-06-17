// includes
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <commons.hpp>

class Camera {
public:
  Camera() : Camera(point3(0, 0, -1), point3(0), vec3(0, 1, 0), 40, 1, 0, 10) {}

  Camera(point3 orig, point3 target, vec3 vup,
         double vfov, // vertical field-of-view in degrees
         double aspect_ratio, double aperture, double focus_dist, double t0 = 0,
         double t1 = 0) {
    origin = orig;
    lens_radius = aperture / 2;
    time0 = t0;
    time1 = t1;

    auto theta = degree_to_radian(vfov);
    auto half_height = tan(theta / 2);
    auto half_width = aspect_ratio * half_height;

    w = unit_vector(orig - target);
    u = unit_vector(cross(vup, w));
    v = cross(w, u);

    lower_left_corner = origin - half_width * focus_dist * u -
                        half_height * focus_dist * v - focus_dist * w;

    horizontal = 2 * half_width * focus_dist * u;
    vertical = 2 * half_height * focus_dist * v;
  }

  Ray get_ray(double s, double t) const {
    vec3 rd = lens_radius * random_in_unit_disk();
    vec3 offset = u * rd.x + v * rd.y;
    vec3 orig_offset = origin + offset;
    vec3 orig_off = origin - offset;
    return Ray(orig_offset,
               lower_left_corner + s * horizontal + t * vertical - orig_off,
               random_double(time0, time1));
  }

private:
  point3 origin;
  point3 lower_left_corner;
  vec3 horizontal;
  vec3 vertical;
  vec3 u, v, w;
  double lens_radius;
  double time0, time1; // shutter open/close times
};

#endif
