#ifndef MATERIAL_HPP
#define MATERIAL_HPP

//
#include <commons.hpp> // done
//
#include <pdf.hpp> // done
//
#include <texture.hpp> // done
//
#include <string>

struct ScatterRecord {
  Ray r_out;
  bool is_specular;
  color attenuation;
  shared_ptr<Pdf> pdf_ptr;
};

class Material {
public:
  virtual color emitted(const Ray &r_in, const HitRecord &rec, double u,
                        double v, const point3 &p) const {
    return color(0);
  }
  virtual bool scatter(const Ray &r_in, const HitRecord &rec,
                       ScatterRecord &srec) const {
    return false;
  }
  virtual double scattering_pdf(const Ray &r_in, const HitRecord &rec,
                                const Ray &scattered) const {
    return 0;
  }
  virtual std::string show_mtype() const { return "material"; }
};
class Dielectric : public Material {
public:
  double ref_idx;

  Dielectric(double r) : ref_idx(r) {}
  std::string show_mtype() const override { return "Dielectric"; }

  double fresnelCT(double costheta, double ridx) const {
    // cook torrence fresnel equation
    double etao = 1 + sqrt(ridx);
    double etau = 1 - sqrt(ridx);
    double eta = etao / etau;
    double g = sqrt(pow(eta, 2) + pow(costheta, 2) - 1);
    double g_c = g - costheta;
    double gplusc = g + costheta;
    double gplus_cc = (gplusc * costheta) - 1;
    double g_cc = (g_c * costheta) + 1;
    double oneplus_gcc = 1 + pow(gplus_cc / g_cc, 2);
    double half_plus_minus = 0.5 * pow(g_c / gplusc, 2);
    return half_plus_minus * oneplus_gcc;
  }
  double fresnelSchlick(double costheta, double ridx) const {
    //
    double r0 = (1 - ridx) / (1 + ridx);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow((1 - costheta), 5);
  }
  double get_fresnel(double costheta, double ridx, int choice = 0) const {
    // compute freshnel
    double fresnel;
    switch (choice) {
    case 0:
      fresnel = fresnelSchlick(costheta, ridx);
      break;
    case 1:
      fresnel = fresnelCT(costheta, ridx);
      break;
    default:
      fresnel = fresnelSchlick(costheta, ridx);
      break;
    }
    return fresnel;
  }
  bool scatter(const Ray &r_in, const HitRecord &rec,
               ScatterRecord &srec) const override {
    srec.is_specular = true;
    srec.pdf_ptr = nullptr;
    srec.attenuation = color(1.0);
    double etai_over_etat = (rec.front_face) ? (1.0 / ref_idx) : (ref_idx);

    vec3 unit_direction = unit_vector(r_in.direction());
    double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
    double sin_theta = sqrt(1.0 - cos_theta * cos_theta);
    if (etai_over_etat * sin_theta > 1.0) {
      vec3 reflected = reflect(unit_direction, rec.normal);
      srec.r_out = Ray(rec.point, reflected, r_in.time());
      return true;
    }

    double reflect_prob = get_fresnel(cos_theta, etai_over_etat);
    if (random_double() < reflect_prob) {
      vec3 reflected = reflect(unit_direction, rec.normal);
      srec.r_out = Ray(rec.point, reflected, r_in.time());
      return true;
    }

    vec3 refracted = refract(unit_direction, rec.normal, etai_over_etat);
    srec.r_out = Ray(rec.point, refracted, r_in.time());
    return true;
  }
};
class DiffuseLight : public Material {
public:
  shared_ptr<Texture> emit;

  std::string show_mtype() const override { return "DiffuseLight"; }
  DiffuseLight(shared_ptr<Texture> a) : emit(a) {}
  color emitted(const Ray &r_in, const HitRecord &rec, double u, double v,
                const point3 &p) const override {
    if (!rec.front_face) {
      return color(0);
    }
    return emit->value(u, v, p);
  }
};
class Isotropic : public Material {
public:
  Isotropic(shared_ptr<Texture> a) : albedo(a) {}

  std::string show_mtype() const override { return "Isotropic"; }
  bool scatter(const Ray &r_in, const HitRecord &rec,
               ScatterRecord &srec) const override {
    srec.r_out = Ray(rec.point, random_in_unit_sphere(), r_in.time());
    srec.attenuation = albedo->value(rec.u, rec.v, rec.point);
    return true;
  }

public:
  shared_ptr<Texture> albedo;
};
class Lambertian : public Material {
public:
  Lambertian(shared_ptr<Texture> a) : albedo(a) {}

  std::string show_mtype() const override { return "Lambertian"; }
  bool scatter(const Ray &r_in, const HitRecord &rec,
               ScatterRecord &srec) const override {
    srec.is_specular = false;
    srec.attenuation = albedo->value(rec.u, rec.v, rec.point);
    srec.pdf_ptr = make_shared<CosinePdf>(rec.normal);
    return true;
  }

  double scattering_pdf(const Ray &r_in, const HitRecord &rec,
                        const Ray &scattered) const override {
    double cosine = dot(rec.normal, to_unit(scattered.direction()));
    return (cosine <= 0) ? 0 : cosine / PI;
  }

public:
  shared_ptr<Texture> albedo;
};
class Metal : public Material {
public:
  Metal(const color &a, double f) : albedo(a), roughness(f < 1 ? f : 1) {}
  std::string show_mtype() const override { return "Metal"; }

  bool scatter(const Ray &r_in, const HitRecord &rec,
               ScatterRecord &srec) const override {
    vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
    srec.r_out = Ray(rec.point, reflected + roughness * random_in_unit_sphere(),
                     r_in.time());
    srec.attenuation = albedo;
    srec.is_specular = true;
    srec.pdf_ptr = nullptr;
    return true;
  }

public:
  color albedo;
  double roughness;
};

#endif
