#pragma once

#include "ray.hh"

namespace RS
{
  class Intersectable;
  /// Intersections are created when a ray hits an intersectable object.
  struct Intersection
  {
    const Intersectable *intersectable;
    Ray incident_ray;
    double t;      // position on ray
    Vec3r position; // position in space
    Vec3r normal;
    Vec2r uv;       // position on renderable
  };
}