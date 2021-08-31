#pragma once

#include <sstream>

#include "aabb.hh"
#include "globals.hh"
#include "types.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  struct DataParams
  {
    //--------------------------------------------------------------------------//
    DataParams(){};
    //--------------------------------------------------------------------------//
    DataParams(const AABB &domain,
               const real &step_size)
        : domain{domain}, step_size{step_size} {}
    //--------------------------------------------------------------------------//
    DataParams(const Vec3r &dMin,
               const Vec3r &dMax,
               const real &step_size)
        : domain(dMin, dMax), step_size{step_size} {}
    //--------------------------------------------------------------------------//
    AABB domain = AABB(Vec3r(-1.0, -1.0, -1.0), Vec3r(1.0, 1.0, 1.0));
    real step_size;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//

  //--------------------------------------------------------------------------//
  struct SearchParams
  {
    //--------------------------------------------------------------------------//
    SearchParams(const real &t0_min,
                 const real &t0_max,
                 const real &tau_min,
                 const real &tau_max,
                 const real &dt,
                 const real &prec)
        : t0_min(t0_min), t0_max(t0_max), tau_min(tau_min), tau_max(tau_max), dt(dt), prec{prec} {}
    //--------------------------------------------------------------------------//
    real t0_min = 0.0;
    real t0_max = 14.8;
    real tau_min = 0.2;
    real tau_max = 15.0;
    real dt = 0.2;
    real prec = Globals::SEARCHPREC;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//
