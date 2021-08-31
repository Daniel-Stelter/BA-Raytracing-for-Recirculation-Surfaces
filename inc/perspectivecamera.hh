#pragma once

#include <cassert>
#include <optional>

#include "camera.hh"
#include "mat.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  enum class CamUp
  {
    Y,
    Z
  };
  //--------------------------------------------------------------------------//
  /// Perspective cameras are able to cast rays from one point called 'eye'
  /// through an image plane.
  ///
  /// Based on the eye position, a look-at point and a field of view angle the
  /// image plane gets constructed.
  /// This camera class constructs a right-handed coordinate system.
  class PerspectiveCamera : public Camera
  {
  private:
    //--------------------------------------------------------------------------//
    Vec3r m_up, m_eye, m_lookat, m_n, m_u, m_v;
    double m_fov, m_plane_distance, m_plane_half_width, m_plane_half_height;
    Vec3r m_bottom_left;
    Vec3r m_plane_base_x, m_plane_base_y;
    mat4 m_M_world_cam;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    /// Constructor generates bottom left image plane pixel position and pixel
    /// offset size.
    PerspectiveCamera(const Vec3r &eye,
                      const Vec3r &lookat,
                      double fov,
                      size_t res_x,
                      size_t res_y,
                      CamUp up_axis);
    //--------------------------------------------------------------------------//
    Ray ray(double x, double y) const override;
    //--------------------------------------------------------------------------//
    Vec2r projection(const Vec3r &pos) const override;
    //--------------------------------------------------------------------------//
    std::shared_ptr<Camera> create_increased(size_t multiplier) const override;
    //--------------------------------------------------------------------------//
    make_clonable(Camera, PerspectiveCamera);
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//