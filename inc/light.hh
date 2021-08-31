#pragma once

#include "clonable.hh"
#include "ray.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class Light : public Clonable<Light>
  {
    //--------------------------------------------------------------------------//
    Vec3r m_spectral_intensity;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    Light() = default;
    //--------------------------------------------------------------------------//
    Light(double r, double g, double b) : m_spectral_intensity{r, g, b} {}
    //--------------------------------------------------------------------------//
    Light(const Vec3r &spectral_intensity)
        : m_spectral_intensity{spectral_intensity} {}
    //--------------------------------------------------------------------------//
    Light(const Light &) = default;
    Light(Light &&) = default;
    virtual ~Light() = default;
    //--------------------------------------------------------------------------//
    /// incident_radiance_at received at a point
    virtual Vec3r incident_radiance_at(const Vec3r &pos) const = 0;
    //--------------------------------------------------------------------------//
    /// returns light direction to a position
    virtual Vec3r light_direction_to(const Vec3r &pos) const = 0;
    //--------------------------------------------------------------------------//
    /// light sources have a spectral intensity that also specifies luminous intensity
    Vec3r spectral_intensity() const { return m_spectral_intensity; }
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//