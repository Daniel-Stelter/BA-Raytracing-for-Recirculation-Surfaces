#pragma once

#include "intersection.hh"
#include <optional>

namespace RS
{
    class Intersectable
    {
    public:
        virtual std::optional<Intersection>
        getIntersection(const Ray &r, double min_t = 0) const = 0;
    };
}