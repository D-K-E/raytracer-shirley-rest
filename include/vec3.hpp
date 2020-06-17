#ifndef VEC3_HPP
#define VEC3_HPP

#include <functions.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <ostream>

using vec3 = glm::dvec3;
using point3 = glm::dvec3;
using color = glm::dvec3;

inline std::ostream &operator<<(std::ostream &out, const vec3 &v) {
  // yazim
  return out << v.x << ' ' << v.y << ' ' << v.z;
}
inline vec3 operator*(const vec3 &v, double d) {
  return vec3(v.x * d, v.y * d, v.z * d);
}
inline vec3 operator*(double d, const vec3 &v) {
  return vec3(v.x * d, v.y * d, v.z * d);
}
inline vec3 operator+(const vec3 &v1, const vec3 &v2) {
  return vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}
inline double dot(const vec3 &u, const vec3 &v) {
  // iccarpim
  return u.x * v.x + u.y * v.y + u.z * v.z;
}
inline vec3 cross(const vec3 &u, const vec3 &v) {
  // capraz carpim
  return vec3(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z,
              u.x * v.y - u.y * v.x);
}
inline vec3 to_unit(vec3 v) { return glm::normalize(v); }
inline vec3 unit_vector(vec3 v) { return to_unit(v); }
inline double length(vec3 v) { return glm::length(v); }
inline double length_squared(vec3 v) { return length(v) * length(v); }
inline static vec3 random_vec() {
  // rastgele vektor uretiyor
  return vec3(random_double(), random_double(), random_double());
}
inline static vec3 random_vec(double min, double max) {
  // rastgele vektor uretiyor
  return vec3(random_double(min, max), random_double(min, max),
              random_double(min, max));
}
inline static vec3 random_in_unit_sphere() {
  // rastgele kurede bir yon veriyor
  while (true) {
    //
    vec3 v = random_vec(-1, 1);
    if (dot(v, v) >= 1) {
      continue;
    }
    return v;
  }
}
inline static vec3 random_unit_vector() {
  // unit vector using spherical coordinates
  double a = random_double(0, 2 * PI);
  double z = random_double(-1, 1);
  double r = sqrt(1 - z * z);
  return vec3(r * cos(a), r * sin(a), z); // sphere coordinates -> xyz
}
inline static vec3 random_in_hemisphere(const vec3 &normal) {
  // normal ekseninde dagilan yon
  vec3 unit_sphere_dir = random_in_unit_sphere();
  if (dot(unit_sphere_dir, normal) > 0.0) {
    return unit_sphere_dir;
  } else {
    return -1 * unit_sphere_dir;
  }
}
vec3 random_in_unit_disk() {
  // lens yakinsamasi için gerekli
  while (true) {
    vec3 point = vec3(random_double(-1, 1), random_double(-1, 1), 0);
    if (dot(point, point) >= 1) {
      continue;
    }
    return point;
  }
}
vec3 reflect(const vec3 &v, const vec3 &n) {
  // yansitma
  return v - 2 * dot(v, n) * n;
}
vec3 refract(const vec3 &uv, const vec3 &n, double eta_over) {
  // isigin bir seyin içinden geçerken kirilmasi
  /*
     eta * sin(theta) = eta' * sin(theta')
     eta = 1.0 hava için
     eta = 1.3 - 1.7 cam için
     eta = 2.4 elmas için
     sin(theta') = eta * sin(theta) / eta'
   */
  double costheta = dot(-1 * uv, n);
  vec3 out_par = eta_over * (uv + costheta * n);
  vec3 out_per = -1 * sqrt(1.0 - dot(out_par, out_par)) * n;
  return out_par + out_per;
}
void to_spheric(const vec3 &v, double &phi, double &theta) {
  // convert components of vec3 to spheric coordinates
  vec3 v2 = to_unit(v);
  phi = atan2(v2.x, v2.z);
  theta = asin(v2.y);
  // phi = atan2(v.z, v.x);
  // theta = asin(v.y);
}
inline vec3 random_cosine_direction() {
  double r1 = random_double();
  double r2 = random_double();
  double z = sqrt(1 - r2);

  double phi = 2 * PI * r1;
  double x = cos(phi) * sqrt(r2);
  double y = sin(phi) * sqrt(r2);

  return vec3(x, y, z);
}

inline vec3 random_to_sphere(double radius, double distance_squared) {
  double r1 = random_double();
  double r2 = random_double();
  double z = 1 + r2 * (sqrt(1 - radius * radius / distance_squared) - 1);

  double phi = 2 * PI * r1;
  double x = cos(phi) * sqrt(1 - z * z);
  double y = sin(phi) * sqrt(1 - z * z);

  return vec3(x, y, z);
}

#endif
