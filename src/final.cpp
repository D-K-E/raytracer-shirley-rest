//
//
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

color ray_color(const Ray &r, const color &background, const Hittable &world,
                shared_ptr<Hittable> lights, int depth) {
  HitRecord rec;

  // If we've exceeded the ray bounce limit, no more light is gathered.
  if (depth <= 0) {
    return color(0);
  }

  // If the ray hits nothing, return the background color.
  if (!world.hit(r, 0.001, INF, rec)) {
    return background;
  }

  ScatterRecord srec;
  color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.point);

  if (!rec.mat_ptr->scatter(r, rec, srec)) {
    dc = emitted;
    return emitted;
  }

  if (srec.is_specular) {
    color rcolor = color(1);
    rcolor *= srec.attenuation;
    rcolor *= ray_color(srec.r_out, background, world, lights, depth - 1);
    dc = rcolor;
    return rcolor;
  }

  auto light_ptr = make_shared<HittablePdf>(lights, rec.point);
  MixturePdf p(light_ptr, srec.pdf_ptr);
  Ray scattered = Ray(rec.point, p.generate(), r.time());
  auto pdf_val = p.value(scattered.direction());
  color rcolor = color(1);

  //
  rcolor *= srec.attenuation;
  rcolor *= rec.mat_ptr->scattering_pdf(r, rec, scattered); // can be 0
  rcolor *= ray_color(scattered, background, world, lights, depth - 1);
  rcolor /= pdf_val; // can be 0 as well resulting in nan
  rcolor += emitted;
  dc = rcolor;

  return rcolor;
}

HittableList cornell_box(Camera &cam, double aspect) {
  HittableList world;

  auto red = make_shared<Lambertian>(make_shared<SolidColor>(.65, .05, .05));
  auto white = make_shared<Lambertian>(make_shared<SolidColor>(.73, .73, .73));
  auto green = make_shared<Lambertian>(make_shared<SolidColor>(.12, .45, .15));
  auto light = make_shared<DiffuseLight>(make_shared<SolidColor>(15, 15, 15));

  world.add(
      make_shared<FlipFace>(make_shared<YZRect>(0, 555, 0, 555, 555, green)));
  world.add(make_shared<YZRect>(0, 555, 0, 555, 0, red));
  world.add(make_shared<FlipFace>(
      make_shared<XZRect>(213, 343, 227, 332, 554, light)));
  world.add(
      make_shared<FlipFace>(make_shared<XZRect>(0, 555, 0, 555, 555, white)));
  world.add(make_shared<XZRect>(0, 555, 0, 555, 0, white));
  world.add(
      make_shared<FlipFace>(make_shared<XYRect>(0, 555, 0, 555, 555, white)));

  shared_ptr<Hittable> box1 =
      make_shared<Box>(point3(0), point3(165, 330, 165), white);
  box1 = make_shared<RotateY>(box1, 15);
  box1 = make_shared<Translate>(box1, vec3(265, 0, 295));
  world.add(box1);

  auto glass = make_shared<Dielectric>(1.5);
  world.add(make_shared<Sphere>(point3(190, 90, 190), 90, glass));

  point3 origin(278, 278, -800);
  point3 target(278, 278, 0);
  vec3 up(0, 1, 0);
  auto dist_to_focus = 10.0;
  auto aperture = 0.0;
  auto vfov = 40.0;
  auto t0 = 0.0;
  auto t1 = 1.0;

  cam =
      Camera(origin, target, up, vfov, aspect, aperture, dist_to_focus, t0, t1);

  return world;
}

int main() {
  const auto aspect_ratio = 16.0 / 9.0;
  const int image_width = 320;
  const int image_height = static_cast<int>(image_width / aspect_ratio);
  const int samples_per_pixel = 100;
  const int max_depth = 50;

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  color background(0, 0, 0);

  Camera cam;
  auto world = cornell_box(cam, aspect_ratio);

  auto lights = make_shared<HittableList>();
  lights->add(
      make_shared<XZRect>(213, 343, 227, 332, 554, shared_ptr<Material>()));
  lights->add(
      make_shared<Sphere>(point3(190, 90, 190), 90, shared_ptr<Material>()));

  for (int j = image_height - 1; j >= 0; --j) {
    std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
    for (int i = 0; i < image_width; ++i) {
      color pixel_color;
      for (int s = 0; s < samples_per_pixel; ++s) {
        auto u = (i + random_double()) / (image_width - 1);
        auto v = (j + random_double()) / (image_height - 1);
        Ray r = cam.get_ray(u, v);
        pixel_color += ray_color(r, background, world, lights, max_depth);
      }
      write_color(std::cout, pixel_color, samples_per_pixel);
    }
  }

  std::cerr << "\nDone.\n";
}
