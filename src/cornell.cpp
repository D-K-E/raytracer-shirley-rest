#include <commons.hpp> // done
//
#include <hittables.hpp> // done
//
#include <camera.hpp> // done
#include <color.hpp>  // done
//
#include <box.hpp> // done

//#include <bvh.hpp>      // done

//#include <mediumc.hpp>  // done

//
#include <filesystem>
#include <iostream>
#include <sphere.hpp> // done
//

color dc = color(0);

color ray_color(const Ray &r, const color &background,
                const HittableList &scene, int depth) {
  // carpan isini renklendirir
  HitRecord record;
  if (depth <= 0) {
    // final case
    return color(0);
  }
  if (!scene.hit(r, 0.001, INF, record)) {
    return background;
  }
  // recursive case
  Ray r_out;
  color atten;
  color emittedColor =
      record.mat_ptr->emitted(record.u, record.v, record.point);
  if (!record.mat_ptr->scatter(r, record, atten, r_out)) {
    return emittedColor;
  }
  // bidirectional surface scattering distribution function
  // rendering equation = L^e + L^r
  return emittedColor + atten * ray_color(r_out, background, scene, depth - 1);
}

HittableList cornell_box() {

  HittableList scene;

  // ---------- materials -----------------
  auto red = make_shared<Lambertian>(make_shared<SolidColor>(0.65, 0.05, 0.05));
  auto white =
      make_shared<Lambertian>(make_shared<SolidColor>(0.88, 0.88, 0.88));
  auto green =
      make_shared<Lambertian>(make_shared<SolidColor>(0.18, 0.78, 0.18));
  auto light =
      make_shared<DiffuseLight>(make_shared<SolidColor>(15.0, 15.0, 15.0));

  // --------- objects -------------------
  scene.add(
      make_shared<FlipFace>(make_shared<YZRect>(0, 555, 0, 555, 555, green)));
  scene.add(make_shared<YZRect>(0, 555, 0, 555, 0, red));
  scene.add(make_shared<XZRect>(213, 343, 227, 332, 554, light));
  scene.add(
      make_shared<FlipFace>(make_shared<XZRect>(0, 555, 0, 555, 0, white)));
  scene.add(make_shared<XZRect>(0, 555, 0, 555, 555, white));
  scene.add(
      make_shared<FlipFace>(make_shared<XYRect>(0, 555, 0, 555, 555, white)));
  scene.add(make_shared<Box>(point3(130, 0, 65), point3(295, 165, 230), white));
  scene.add(
      make_shared<Box>(point3(265, 0, 295), point3(430, 330, 460), white));

  return scene;
}

struct InnerParams {
  TimeRayCamera camera;
  int imwidth;
  int imheight;
  int psample;
  int mdepth;
  int imi;
  int imj;
  HittableList scene;
};

void innerLoop(InnerParams params) {
  // inner loop for write_color
  TimeRayCamera camera = params.camera;
  int imwidth = params.imwidth;
  int imheight = params.imheight;
  int psample = params.psample;
  int mdepth = params.mdepth;
  int i = params.imi;
  int j = params.imj;
  HittableList scene = params.scene;
  color background(0);
  //
  color rcolor(0.0, 0.0, 0.0);
  for (int k = 0; k < psample; k++) {
    double t = double(i + random_double()) / (imwidth - 1);
    double s = double(j + random_double()) / (imheight - 1);
    Ray r = camera.get_ray(t, s);
    rcolor += ray_color(r, background, scene, mdepth);
  }
  write_color(std::cout, rcolor, psample);
}

int main(void) {
  // resmin yazma fonksiyonu
  double aspect_ratio = 16.0 / 9.0;
  const int imwidth = 320;
  const int imheight = static_cast<int>(imwidth / aspect_ratio);
  int psample = 100;
  int mdepth = 50;

  // ppm için gerekli olanlar
  std::cout << "P3" << std::endl;
  std::cout << imwidth << ' ' << imheight << std::endl;
  std::cout << "255" << std::endl;

  // konacak objelerin deklarasyonu
  HittableList scene = cornell_box();

  // kamera
  vec3 vup(0, 1, 0);
  double dist_to_focus = 10.0;
  double aperature = 0.1;
  TimeRayCamera camera(point3(278, 278, -800), point3(278, 278, 0), vup, 40,
                       aspect_ratio, aperature, dist_to_focus, 0.0, 1.0);

  // resim yazim
  for (int j = imheight - 1; j >= 0; j -= 1) {
    std::cerr << "\rKalan Tarama Çizgisi:" << ' ' << j << ' ' << std::flush;
    for (int i = 0; i < imwidth; i += 1) {
      InnerParams params1 = {camera, imwidth, imheight, psample,
                             mdepth, i,       j,        scene};
      innerLoop(params1);
      //
    }
  }
}
