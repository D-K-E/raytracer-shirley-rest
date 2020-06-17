#ifndef IMAGE_HPP
#define IMAGE_HPP
// helper image class
//
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
//
#include <commons.hpp>
#include <vector>

class Image {

public:
  int width, height;
  int nbChannels;
  std::vector<std::vector<color>> imdata;
  unsigned char *data;

  Image() : width(0), height(0), nbChannels(0) {}
  Image(const char *filename) {
    // load file
    // basic support for png and jpeg
    nbChannels = strstr(filename, "png") ? 4 : 3;
    data = stbi_load(filename, &width, &height, &nbChannels, nbChannels);
    if (data == nullptr) {
      std::cerr << "Could not load image from " << filename << std::endl;
      width = height = 0;
    }
    load(data, width, height, nbChannels);
  }
  void load(unsigned char *data, int width, int height, int nbChannels) {
    // load data
    int dsize = nbChannels * width * height;
    std::vector<color> imvec(width * height);
    int s = 0;
    imdata = std::vector<std::vector<color>>(width, std::vector<color>(height));
    for (int i = 0; i < dsize; i += nbChannels) {
      color pcolor; // our color vector have 3 components; so no alpha
      pcolor[0] = double(data[i]);
      pcolor[1] = double(data[i + 1]);
      pcolor[2] = double(data[i + 2]);
      imvec[s] = pcolor;
      s += 1;
    }
    for (int i = 0; i < width; i++) {
      for (int k = 0; k < height; k++) {
        imdata[i][k] = imvec[i * k];
      }
    }
  }
  void save(const char *path) const {
    // save image to somewhere
    if (nbChannels == 3) {
      //
      stbi_write_jpg(path, width, height, nbChannels, data, 100);
    } else {
      stbi_write_png(path, width, height, nbChannels, data, width * nbChannels);
    }
  }
  void clear() { stbi_image_free(data); }
  color getPixelColor(int w, int h) const {
    //
    return imdata[w][h];
  }
  color getPixelValue(int x, int y) const {
    //
    unsigned char *pixel = data + (x + width * y) * nbChannels;
    color pcolor(pixel[0], pixel[1], pixel[2]);
    return pcolor;
  }
  int rows() const { return height; }
  int cols() const { return width; }
  int h() const { return height; }
  int w() const { return width; }
};

#endif
