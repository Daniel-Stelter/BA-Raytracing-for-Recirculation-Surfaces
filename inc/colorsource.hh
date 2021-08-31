#pragma once

#include "clonable.hh"
#include "types.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class ColorSource : public Clonable<ColorSource>
  {
  public:
    //--------------------------------------------------------------------------//
    virtual Vec3r sample(double u, double v) const = 0;
    auto sample(const Vec2r &uv) const { return sample(uv[0], uv[1]); }
    auto operator()(const Vec2r &uv) const { return sample(uv[0], uv[1]); }
    auto operator()(double u, double v) const { return sample(u, v); }
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//