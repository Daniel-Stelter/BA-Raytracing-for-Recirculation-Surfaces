#define _USE_MATH_DEFINES
#include "perspectivecamera.hh"

#include <cmath>
#include "vec.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  PerspectiveCamera::PerspectiveCamera(const Vec3r &eye,
                                       const Vec3r &lookat,
                                       double fov,
                                       size_t res_x,
                                       size_t res_y,
                                       CamUp up_axis)
      : Camera{res_x, res_y},
        m_up{},
        m_eye{eye},
        m_lookat{lookat},
        m_n{(eye - lookat).normalize()},
        m_u{},
        m_v{},
        m_fov{fov},
        m_plane_distance{(lookat - eye).norm()},
        m_plane_half_width{std::tan(fov * M_PI / 180)},
        m_plane_half_height{double(res_y) / double(res_x) * m_plane_half_width},
        m_bottom_left{},
        m_plane_base_x{},
        m_plane_base_y{},
        m_M_world_cam{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}
  {
    m_up = (up_axis == CamUp::Y) ? Vec3r{0, 1, 0} : Vec3r{0, 0, 1};
    m_u = (m_up % m_n); // cross product
    m_v = (m_n % m_u);  // cross product
    m_bottom_left = lookat - m_u * m_plane_half_width - m_v * m_plane_half_height;
    m_plane_base_x = m_u * (2 * m_plane_half_width / res_x);
    m_plane_base_y = m_v * (2 * m_plane_half_height / res_y);

    mat4 R = (up_axis == CamUp::Y)
                 ? mat4({m_u[0], m_v[0], m_n[0], 0.},
                        {m_u[1], m_v[1], m_n[1], 0.},
                        {m_u[2], m_v[2], m_n[2], 0.},
                        {0., 0., 0., 1.})
                 : mat4({m_u[0], m_n[0], m_v[0], 0.},
                        {m_u[1], m_n[1], m_v[1], 0.},
                        {m_u[2], m_n[2], m_v[2], 0.},
                        {0., 0., 0., 1.});
    optional<mat4> R_inv = inverse(R);
    mat4 T{{1., 0., 0., m_eye[0]},
           {0., 1., 0., m_eye[1]},
           {0., 0., 1., m_eye[2]},
           {0., 0., 0., 1.}};
    optional<mat4> T_inv = inverse(T);
    if (!R_inv || !T_inv)
      cerr << "Camera orientation is invalid" << endl;
    else
      m_M_world_cam = R_inv.value() * T_inv.value();
  }

  //--------------------------------------------------------------------------//
  Ray PerspectiveCamera::ray(double x, double y) const
  {
    const auto view_plane_point =
        m_bottom_left + m_plane_base_x * x + m_plane_base_y * y;
    return {{m_eye}, {view_plane_point - m_eye}};
  }

  //--------------------------------------------------------------------------//
  Vec2r PerspectiveCamera::projection(const Vec3r &pos) const
  {
    // transformation to cam system
    vec4 vec = m_M_world_cam * vec4{pos[0], pos[1], pos[2], 1};

    // project pos to the canvas
    real v_u = vec(0);
    real v_v = m_up[1] == 1 ? vec(1) : vec(2);
    real v_n = m_up[1] == 1 ? vec(2) : vec(1);
    real ratio = -m_plane_distance / v_n;
    Vec2r result{v_u * ratio, v_v * ratio};

    // currently canvas goes from (-width/2,-height/2) to (width/2,height/2)
    result += Vec2r{m_plane_half_width, m_plane_half_height};
    return Vec2r{result[0] * plane_width() / (m_plane_half_width * 2),
                 result[1] * plane_height() / (m_plane_half_height * 2)};
  }

  //--------------------------------------------------------------------------//
  shared_ptr<Camera> PerspectiveCamera::create_increased(size_t multiplier) const
  {
    CamUp cam_up = m_up[1] == 1 ? CamUp::Y : CamUp::Z;
    return make_shared<PerspectiveCamera>(m_eye,
                                          m_lookat,
                                          m_fov,
                                          plane_width() * multiplier,
                                          plane_height() * multiplier,
                                          cam_up);
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//