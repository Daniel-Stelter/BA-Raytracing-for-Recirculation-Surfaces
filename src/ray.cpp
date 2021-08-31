#include "ray.hh"

namespace RS
{
  std::optional<Vec3r> refract(const Vec3r &incident, Vec3r n, double ior)
  {
    n.normalize();
    double cosi = (Vec3r(incident).normalize() | n);
    double etai = 1, etat = ior;
    if (cosi < 0)
    {
      cosi = -cosi;
    }
    else
    {
      std::swap(etai, etat);
      n = -n;
    }
    double eta = etai / etat;
    double k = 1 - eta * eta * (1 - cosi * cosi);
    if (k < 0)
    {
      return {};
    }
    return incident * eta + n * (eta * cosi - std::sqrt(k));
  }
}