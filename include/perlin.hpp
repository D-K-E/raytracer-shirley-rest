#ifndef PERLIN_HPP
#define PERLIN_HPP
//
#include <commons.hpp>
//
class Perlin {
public:
  Perlin() {
    //
    ranvec = new vec3[point_count];
    for (int i = 0; i < point_count; ++i) {
      ranvec[i] = to_unit(random_vec(-1, 1));
    }
    perm_x = perlin_generate_perm();
    perm_y = perlin_generate_perm();
    perm_z = perlin_generate_perm();
  }
  ~Perlin() {
    delete[] ranvec;
    delete[] perm_x;
    delete[] perm_y;
    delete[] perm_z;
  }
  double noise(const point3 &p) const {
    // generate noise
    double u, v, w;
    // Hermetian smoothing
    u = p.x - floor(p.x);
    v = p.y - floor(p.y);
    w = p.z - floor(p.z);

    auto indexfn = [](int f, int s) { return (f + s) & 255; };

    int i, j, k;
    i = static_cast<int>(floor(p.x));
    j = static_cast<int>(floor(p.y));
    k = static_cast<int>(floor(p.z));
    vec3 c[2][2][2];
    int arr[2] = {0, 1};
    for (const int a : arr) {
      for (const int b : arr) {
        for (const int d : arr) {
          c[a][b][d] = ranvec[perm_x[indexfn(i, a)] xor
                              perm_y[indexfn(j, b)] xor perm_z[indexfn(k, d)]];
        }
      }
    }
    return perlin_interp(c, u, v, w);
  }
  double turb(const point3 &p, int depth = 7) const {
    // turbulance
    point3 temp_point = p;
    double acc = 0.0;
    double weight = 1.0;
    //
    for (int i = 0; i < depth; i++) {
      acc += weight * noise(temp_point);
      weight *= 0.5;
      temp_point *= 2;
    }
    return fabs(acc);
  };

private:
  static const int point_count = 256;
  vec3 *ranvec;
  int *perm_x; // integer array pointer
  int *perm_y;
  int *perm_z;

  static int *perlin_generate_perm() {
    // perlin generate permutate
    int *p = new int[point_count];

    for (int i = 0; i < point_count; i++) {
      p[i] = i;
    }
    permute(p, point_count);
    return p;
  }
  static void permute(int *p, // integer array pointer
                      int n) {
    //
    for (int i = n - 1; i > 0; i--) {
      auto target = random_int(0, i);
      int tmp = p[i];
      p[i] = p[target];
      p[target] = tmp;
    }
  }
  inline double perlin_interp(vec3 c[2][2][2], double u, double v,
                              double w) const {
    // trilinear interpolation with weight factor
    // http://paulbourke.net/miscellaneous/interpolation/
    auto uu = u * u * (3 - 2 * u);
    auto vv = v * v * (3 - 2 * v);
    auto ww = w * w * (3 - 2 * w);

    double a = 0.0;
    int arr[2] = {0, 1};
    for (const int i : arr) {
      for (const int j : arr) {
        for (const int k : arr) {
          vec3 weight = vec3(u - i, v - j, w - k);
          a += (i * uu + (1 - i) * (1 - uu)) * (j * vv + (1 - j) * (1 - vv)) *
               (k * ww + (1 - k) * (1 - ww)) * dot(c[i][j][k], weight);
        }
      }
    }
    return a;
  }
};
#endif
