#ifndef TEXTURE_HPP
#define TEXTURE_HPP
#include <commons.hpp>
#include <perlin.hpp>
//
#include <image.hpp>
//
// ------------ texture utils ----------------------

void get_sphere_uv(const vec3 &outNormal, double &u, double &v) {
  // http://www.cse.msu.edu/~cse872/tutorial4.html
  /*
The interesting problem for the sphere is to determine the texture coordinates
for each vertex.  One way to look at this problem is to use the surface normal
as way to tell where we are on the face of the sphere.  If we consider this a
vector from the inside of the sphere, we can easily compute the latitude and
longitude on the surface of the globe.  These values correspond to point on the
map.

Right after the call to glBegin(GL_TRIANGLES), add the following code:

  // What's the texture coordinate for this normal?
  tx1 = atan2(a[0], a[2]) / (2. * GR_PI) + 0.5;
  ty1 = asin(a[1]) / GR_PI + .5;

  glTexCoord2f(tx1, ty1);

So, what does this do?  a[0] and a[2] are the X and Z values of the normal.  If
you look straight down onto the globe from the top, the vector made up of the X
and Z values will tell you the longitude on the globe!  I use atan2 to convert
that to an angle in radians.  This angle is between -PI and PI.  I divide by
2PI, so it's now between -.5 and .5.  Adding 0.5 makes it range from 0 to 1.
This is the X value in the texture map.
   */
  /*
double phi = atan2(outNormal.x, outNormal.z);
double theta = asin(outNormal.y);
u = 0.5 + (phi / 2 * PI);
v = 0.5 + (theta / PI);
*/
  auto phi = atan2(outNormal.z, outNormal.x);
  auto theta = asin(outNormal.y);
  u = 1 - (phi + PI) / (2 * PI);
  v = (theta + PI / 2) / PI;
}

// -------------------------------------------------

class Texture {
public:
  virtual color value(double u, double v, const point3 &p) const = 0;
};

class SolidColor : public Texture {
public:
  color color_value;

public:
  SolidColor() {}
  SolidColor(color c) : color_value(c) {}
  SolidColor(double red, double green, double blue)
      : color_value(color(red, green, blue)) {}
  virtual color value(double u, double v, const point3 &p) const {
    // std::cerr << "solid color: " << color_value << std::endl;
    return color_value;
  }
};
class CheckerTexture : public Texture {
private:
  shared_ptr<Texture> odd;
  shared_ptr<Texture> even;

public:
  CheckerTexture() {}
  CheckerTexture(shared_ptr<Texture> t1, shared_ptr<Texture> t2)
      : odd(t1), even(t2) {}
  virtual color value(double u, double v, const point3 &p) const {
    double sines = sin(10 * p.x) * sin(10 * p.y) * sin(10 * p.z);
    if (sines < 0) {
      return odd->value(u, v, p);
    } else {
      return even->value(u, v, p);
    }
  }
};
class NoiseTexture : public Texture {
public:
  NoiseTexture() {}
  NoiseTexture(double sc) : scale(sc) {}
  virtual color value(double u, double v, const point3 &p) const {
    auto tmp = 1.0 + sin(scale * p.z + 10 * noise.turb(p));
    return color(1) * 0.5 * tmp;
  }

public:
  Perlin noise;
  double scale;
};
class ImageTexture : public Texture {
  /*
     Texture mapping and sampling
So instead, one of the the most universal unofficial standards in graphics is to
use texture coordinates instead of image pixel coordinates. These are just some
form of fractional position in the image. For example, for pixel (i,j) in an Nx
by Ny image, the image texture position is:

u=i / Nx−1
v=j / Ny−1

This is just a fractional position. For a hittable, we need to also return a u
and v in the hit record.

For spheres:
u = phi / 2pi
v = theta / pi
phi = atan2(y,x)
theta = asin(z)
   */
private:
  int width, height;
  Image img;

public:
  ImageTexture() : width(0), height(0) {}
  ImageTexture(const char *filename) {
    // simple support for png files as well
    img = Image(filename);
    width = img.w();
    height = img.h();
  }
  virtual color value(double u, double v, const vec3 &p) const {
    //
    if (img.w() == 0 && img.h() == 0) {
      // image not loaded is returned as cyan for debugging
      std::cerr << "imdata empty" << std::endl;
      return color(0.0, 1, 0);
    }
    // clamp input texture coordinates [0,1] x [1,0]
    u = clamp(u, 0.0, 1.0);
    v = 1.0 - clamp(v, 0.0, 1.0);

    int x = static_cast<int>(u * width);
    int y = static_cast<int>(v * height);
    if (x >= width) {
      x = width - 1;
    }
    if (y >= height) {
      y = height - 1;
    }
    color imc = img.getPixelValue(x, y) / 255.0;
    // std::cerr << "w: " << w << " h: " << h << " c: " << imc << std::endl;
    // std::cerr << "w: " << w << " h: " << h << " c: " << imc << " p: " << p
    //          << " u: " << u << " v: " << v << std::endl;
    return imc;
  }
};

#endif
