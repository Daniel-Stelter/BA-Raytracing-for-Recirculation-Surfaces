#pragma once

#include "light.hh"
#include "utils.hh"
#include "vclibs/math/VecN.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class DirectionalLight : public Light
  {
    //--------------------------------------------------------------------------//
    Vec3r m_direction;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    DirectionalLight(const Vec3r &dir)
        : m_direction{dir} { m_direction.normalize(); }
    //--------------------------------------------------------------------------//
    DirectionalLight(const Vec3r &dir, const Vec3r &spectral_intensity)
        : Light{spectral_intensity}, m_direction{dir} { m_direction.normalize(); }
    //--------------------------------------------------------------------------//
    Vec3r incident_radiance_at(const Vec3r &pos) const override
    {
      utils::unusedArgs(pos);
      return spectral_intensity();
    }
    //--------------------------------------------------------------------------//
    Vec3r light_direction_to(const Vec3r &pos) const override
    {
      utils::unusedArgs(pos);
      return m_direction;
    }
    //--------------------------------------------------------------------------//
    make_clonable(Light, DirectionalLight);
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//