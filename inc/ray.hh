#pragma once

#include <optional>
#include "types.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  /// reflects a vector d depending on a normal vector.
  inline Vec3r reflect(const Vec3r &d, const Vec3r &n)
  {
    return (d - n * (2 * (d | n)));
  }
  //--------------------------------------------------------------------------//
  /// calculates refraction vector with incident direction, normal direction and
  /// index of refration ior.
  std::optional<Vec3r> refract(const Vec3r &incident, Vec3r n, double ior);
  //--------------------------------------------------------------------------//
  /// Rays have an origin and a direction.
  /// They are used for intersection tests with \ref RS::Renderable "renderables".
  class Ray
  {
    //--------------------------------------------------------------------------//
    Vec3r m_origin, m_direction;
    size_t m_num_reflections;
    //--------------------------------------------------------------------------//
    Ray(const Vec3r &org, const Vec3r &dir, size_t num_reflections)
        : m_origin{org},
          m_direction{dir},
          m_num_reflections{num_reflections} { m_direction.normalize(); }
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    Ray(const Vec3r &org, const Vec3r &dir)
        : m_origin{org}, m_direction{dir}, m_num_reflections{0} { m_direction.normalize(); }
    //--------------------------------------------------------------------------//
    Ray(const Ray &) = default;
    Ray(Ray &&) = default;
    Ray &operator=(const Ray &) = default;
    Ray &operator=(Ray &&) = default;
    //--------------------------------------------------------------------------//
    /// Evaluate the ray at position t.
    /// t = 1 returns origin + normalized direction.
    Vec3r operator()(double t) const
    {
      return m_origin + m_direction * t;
    }
    //--------------------------------------------------------------------------//
    const auto &origin() const { return m_origin; }
    auto origin(size_t i) const { return m_origin[i]; }
    //--------------------------------------------------------------------------//
    const auto &direction() const { return m_direction; }
    auto direction(size_t i) const { return m_direction[i]; }
    //--------------------------------------------------------------------------//
    auto num_reflections() const { return m_num_reflections; }
    //--------------------------------------------------------------------------//
    /// Reflects the ray at some position with normal.
    auto reflect(const Vec3r &position, const Vec3r &normal) const
    {
      const auto dir = RS::reflect(m_direction, normal);
      return Ray{position + dir * 1e-4, dir, m_num_reflections + 1};
    }
    //--------------------------------------------------------------------------//
    /// calculates refracted ray at some position with normal and index of
    /// refration ior.
    std::optional<Ray> refract(const Vec3r &position, const Vec3r &normal,
                               double ior) const
    {
      const auto dir = RS::refract(m_direction, normal, ior);
      if (!dir)
      {
        return {};
      }
      return Ray{position + *dir * 1e-4, *dir, m_num_reflections + 1};
    }
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//