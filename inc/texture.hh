#pragma once

#include <array>
#include <string>
#include <cmath>
#include <vector>
#include "colors.hh"
#include "colorsource.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class Texture : public ColorSource
  {
    //--------------------------------------------------------------------------//
    std::array<size_t, 2> m_resolution;
    std::vector<Vec3r> m_pixel_data;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    Texture(const std::string &filepath);
    Texture(size_t res_u, size_t res_v, const color &col = white);
    Texture(size_t res_u, size_t res_v, const std::vector<Vec3r> &data_pixel);
    //--------------------------------------------------------------------------//
    Texture(const Texture &) = default;
    Texture(Texture &&) = default;
    //--------------------------------------------------------------------------//
    Texture &operator=(const Texture &) = default;
    Texture &operator=(Texture &&) = default;
    //--------------------------------------------------------------------------//
    size_t res_u() const { return m_resolution[0]; }
    size_t res_v() const { return m_resolution[1]; }
    //--------------------------------------------------------------------------//
    /// Samples texture with bilinear interpolation.
    Vec3r sample(double u, double v) const override;
    //--------------------------------------------------------------------------//
    /// returns pixel at position [u,v] with read/write access
    Vec3r &pixel(size_t u, size_t v);
    //--------------------------------------------------------------------------//
    /// returns pixel at position [u,v] with read-only access
    const Vec3r &pixel(size_t u, size_t v) const;
    //--------------------------------------------------------------------------//
    /// reads image data data structre
    void read_ppm(const std::string &filepath);
    //--------------------------------------------------------------------------//
    /// writes image data into a file with ppm format
    void write_ppm(const std::string &filepath) const;
    //--------------------------------------------------------------------------//
    make_clonable(ColorSource, Texture);
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//