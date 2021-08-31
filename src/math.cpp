#include "math.hh"

namespace RS
{
  Vec3r surface_normal(const Vec3r &v1, const Vec3r &v2, const Vec3r &v3)
  {
    // directions on surface
    Vec3r d1 = v1 - v2;
    Vec3r d2 = v1 - v3;
    // check linear dependence
    if (std::abs(d1.norm()) <= Globals::ZERO || std::abs(d2.norm()) <= Globals::ZERO)
      return Vec3r{0, 0, 0};
    d1.normalize();
    d2.normalize();
    Vec3r result = (d1 % d2);
    real len = result.norm();
    if (len < Globals::EPS)
      return Vec3r{0, 0, 0};
    // calculate normal
    return result / len;
  }
}