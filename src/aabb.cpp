#include "aabb.hh"

#include "globals.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
    // ------------------------------------------------------------------------- //
    // helper function
    void swapValue(Vec3r &v1, Vec3r &v2, size_t i)
    {
        real temp = v1[i];
        v1[i] = v2[i];
        v2[i] = temp;
    }
    // ------------------------------------------------------------------------- //
    AABB::AABB(const Vec3r &v1, const Vec3r &v2)
        : v_min{v1},
          v_max{v2}
    {
        for (size_t i = 0; i < 3; ++i)
            if (v_min[i] > v_max[i])
                swapValue(v_min, v_max, i);
    }
    // ------------------------------------------------------------------------- //
    bool AABB::isInside(const Vec3r &v) const
    {
        for (size_t i = 0; i < 3; ++i)
            if (v[i] < v_min[i] || v[i] > v_max[i])
                return false;
        return true;
    }
    // ------------------------------------------------------------------------- //
    std::optional<Vec2r> AABB::getIntersectionPositions(const Ray &ray,
                                                        size_t &hit_in,
                                                        const real &min_t,
                                                        const real &max_t) const
    {
        // special case: ray is parallel to (at least) one axis
        bool parallel[3]{false, false, false};
        for (size_t i = 0; i < 3; i++)
        {
            if (abs(ray.direction()[i]) < Globals::EPS)
            {
                parallel[i] = true;
                if (ray.origin()[i] < v_min[i] || ray.origin()[i] > v_max[i])
                    return {};
            }
        }
        // calculate position of intersection for all three dimensions
        Vec3r vEnter = v_min - ray.origin();
        Vec3r vExit = v_max - ray.origin();

        for (size_t i = 0; i < 3; i++)
        {
            if (!parallel[i])
            {
                vEnter[i] /= ray.direction()[i];
                vExit[i] /= ray.direction()[i];
                if (vEnter[i] > vExit[i])
                    swapValue(vEnter, vExit, i);
            }
            else
            {
                vEnter[i] = numeric_limits<real>::min();
                vExit[i] = numeric_limits<real>::max();
            }
        }
        real i_min = std::max({vEnter[0], vEnter[1], vEnter[2], min_t});
        real i_max = std::min({vExit[0], vExit[1], vExit[2], max_t});

        if (i_min >= i_max)
            return {};
        for (size_t i = 0; i < 3; ++i)
            if (i_min == vEnter[i])
                hit_in = i;
        return Vec2r{i_min, i_max};
    }
    // ------------------------------------------------------------------------- //
    std::optional<Vec2r> AABB::getIntersectionPositions(const Ray &ray,
                                                        real min_t,
                                                        real max_t) const
    {
        size_t unused;
        return getIntersectionPositions(ray, unused, min_t, max_t);
    }
    // ------------------------------------------------------------------------- //
    std::optional<Intersection> AABB::getIntersection(const Ray &ray,
                                                      real min_t) const
    {
        // if inside: no normal
        if (isInside(ray(min_t)))
            return Intersection{nullptr, ray, min_t, ray(min_t), Vec3r{0, 0, 0}, Vec2r{0, 0}};

        size_t hit_in = 3;
        // if not intersecting: return empty
        auto opt = getIntersectionPositions(ray, hit_in, min_t);
        if (!opt.has_value())
            return {};
        real &t = opt.value()[0];
        Vec3r pos = ray(t);

        // create normal
        assert(hit_in < 3);
        Vec3r normal{0, 0, 0};
        // if the ray direction is positive for corresponding coord, then
        // the normal must be negative
        normal[hit_in] = ray.direction(hit_in) > 0 ? -1 : 1;

        // find other coordinates
        Vec2r uv{};
        size_t uv_pos = 0;
        // iterate over 3 ordiantes, but leave hit_in out
        for (size_t i = 0; i < 3; ++i)
            if (i != hit_in)
                uv[uv_pos++] = (pos[i] - v_min[i]) / (v_max[i] - v_min[i]);

        return Intersection{nullptr, ray, t, pos, normal, uv};
    }
    // ------------------------------------------------------------------------- //
}
//--------------------------------------------------------------------------//