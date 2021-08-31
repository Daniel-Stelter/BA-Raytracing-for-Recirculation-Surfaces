#pragma once

#include "aabb.hh"
#include "colors.hh"
#include "renderable.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    /// Renderable AABB
    class Box : public Renderable
    {
    private:
        //--------------------------------------------------------------------------//
        AABB m_aabb;
        //--------------------------------------------------------------------------//
    public:
        //--------------------------------------------------------------------------//
        /// Constructor with an AABB and a Phong for rendering
        Box(const AABB &aabb,
            const Phong &phong = Phong{gray, 0.8, 0.4, 0.3, 3})
            : Renderable{phong}, m_aabb{aabb} {}
        //--------------------------------------------------------------------------//
        /// Constructor with two vectors which span an AABB and a Phong for rendering
        Box(const Vec3r &v1,
            const Vec3r &v2,
            const Phong &phong = Phong{gray, 0.8, 0.4, 0.3, 3})
            : Renderable{phong}, m_aabb{v1, v2} {}
        //--------------------------------------------------------------------------//
        /// Search intersection to the box
        std::optional<Intersection> getIntersection(const Ray &ray,
                                                    double min_t = 0) const
        {
            auto opt = m_aabb.getIntersection(ray, min_t);
            if (opt)
                opt->intersectable = this;
            return opt;
        }
        //--------------------------------------------------------------------------//
        make_clonable(Renderable, Box);
        //--------------------------------------------------------------------------//
    };
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//