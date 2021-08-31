#pragma once

#include "globals.hh"
#include "types.hh"
#include <sstream>

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class RecPoint
  {
  public:
    //--------------------------------------------------------------------------//
    RecPoint(){};
    //--------------------------------------------------------------------------//
    RecPoint(const real &x,
             const real &y,
             const real &z,
             const real &t0,
             const real &tau)
        : pos(Vec3r(x, y, z)), t0(t0), tau(tau) {}
    //--------------------------------------------------------------------------//
    RecPoint(Vec3r pos, real t0, real tau)
        : pos(pos), t0(t0), tau(tau) {}
    //--------------------------------------------------------------------------//
    RecPoint(Vec5r vec)
        : pos(Vec3r(vec[0], vec[1], vec[2])), t0(vec[3]), tau(vec[4]) {}
    //--------------------------------------------------------------------------//
    RecPoint(Vec6r vec)
        : pos(Vec3r(vec[0], vec[1], vec[2])), t0(vec[3]), tau(vec[4]), dist(vec[5]) {}
    //--------------------------------------------------------------------------//
    RecPoint(const Vec3r &pos,
             const Vec3r &n,
             const real &t0,
             const real &tau,
             real const &dist)
        : pos(pos), n(n), t0(t0), tau(tau), dist(dist) {}
    //--------------------------------------------------------------------------//
    operator Vec5r(void) const { return Vec5r(pos[0], pos[1], pos[2], t0, tau); }
    //--------------------------------------------------------------------------//
    operator Vec6r(void) const { return Vec6r(pos[0], pos[1], pos[2], t0, tau, dist); }
    //--------------------------------------------------------------------------//
    real &operator[](const unsigned int &index)
    {
      if (3 > index)
        return pos[index];
      if (index == 3)
        return t0;
      if (index == 4)
        return tau;
      if (index == 5)
        return dist;
      throw std::out_of_range("out of bound");
    }
    //--------------------------------------------------------------------------//
    bool operator<(const RecPoint &point) const
    {
      Vec3r dV = pos - point.pos;
      real dt0 = abs(t0 - point.t0);
      real rcpDist =
          sqrt(dV[0] * dV[0] + dV[1] * dV[1] + dV[2] * dV[2] + dt0 * dt0);

      if (rcpDist < Globals::RECPOINTEQUAL)
        return false;

      real ptDist = dV.norm2();
      CmpVec3r cv;
      if (ptDist > Globals::SPACEEQUAL)
        return cv(pos, point.pos);
      if (dt0 > Globals::T0EQUAL)
        return (t0 < point.t0);
      return false;
    }
    //--------------------------------------------------------------------------//
    /** Compute the Euclidean distance to another RecPoint. */
    real distance(const RecPoint &point)
    {
      Vec3r dV = pos - point.pos;
      real dt0 = abs(t0 - point.t0);
      return sqrt(dV[0] * dV[0] + dV[1] * dV[1] + dV[2] * dV[2] + dt0 * dt0);
    }
    //--------------------------------------------------------------------------//
    RecPoint &operator=(const RecPoint &point)
    {
      t0 = point.t0;
      tau = point.tau;
      pos = point.pos;
      n = point.n;
      dist = point.dist;
      sId = point.sId;
      return *this;
    }
    //--------------------------------------------------------------------------//
    std::string toString(void) const
    {
      std::stringstream os;
      os << "[" << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << t0
         << ", " << tau << "]";
      return os.str();
    }
    //--------------------------------------------------------------------------//
    Vec3r pos = Vec3r(0.0, 0.0, 0.0);
    Vec3r n = Vec3r(0.0, 0.0, 0.0);
    real t0 = 0.0;
    real tau = 0.0;
    real dist =
        std::numeric_limits<double>::max(); // distance between t0- & tau-position
    unsigned int sId =
        0; // index of the sampling position that was used to find this RecPoint
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//