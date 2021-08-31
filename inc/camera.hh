#pragma once

#include <memory>
#include "clonable.hh"
#include "ray.hh"
//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  /// \brief Interface for camera implementations.
  ///
  /// Implementations must override the ray method that casts rays through the
  /// camera's image plane.
  class Camera : public Clonable<Camera>
  {
    //--------------------------------------------------------------------------//
    const std::array<size_t, 2> m_resolution;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    Camera(size_t res_x, size_t res_y) : m_resolution{res_x, res_y} {}
    virtual ~Camera() = default;
    //--------------------------------------------------------------------------//
    /// Returns number of pixels of plane in x-direction.
    size_t plane_width() const { return m_resolution[0]; }
    //--------------------------------------------------------------------------//
    /// Returns number of pixels of plane in y-direction.
    size_t plane_height() const { return m_resolution[1]; }
    //--------------------------------------------------------------------------//
    ///  Gets a ray through plane at pixel with coordinate [x,y].
    /// [0,0] is bottom left.
    /// Ray goes through center of pixel.
    /// This method must be overridden in camera implementations.
    virtual Ray ray(double x, double y) const = 0;
    //--------------------------------------------------------------------------//
    /// Maps a point in space to the position of the canvas.
    /// This method must be overridden in camera implementations.
    virtual Vec2r projection(const Vec3r &pos) const = 0;
    //--------------------------------------------------------------------------//
    /// Creates a camera with the same settings as of this one,
    /// but with increased resolution.
    virtual std::shared_ptr<Camera> create_increased(size_t multiplier) const = 0;
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//