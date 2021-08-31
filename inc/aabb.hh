#pragma once

#include "intersectable.hh"
#include "phong.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    // ------------------------------------------------------------------------- //
    /// AABB (axis alligned bounding box):
    /// Classical object which defines a cylinder with parallel edges
    /// to the axes. Box is spanned by two points.
    class AABB : public Intersectable
    {
    private:
        // ------------------------------------------------------------------------- //
        Vec3r v_min, v_max;
        // ------------------------------------------------------------------------- //
        /// Returns an optional 2D vector which contains the ray positions
        /// of the entering and the exiting positions. If there is no intersection
        /// then the optional does not contain an object.
        /// Gives also information about which coordiante was intersected.
        std::optional<Vec2r> getIntersectionPositions(const Ray &ray,
                                                      size_t &hit_in,
                                                      const real &min_t = 0.0,
                                                      const real &max_t = std::numeric_limits<real>::max()) const;
        // ------------------------------------------------------------------------- //
    public:
        // ------------------------------------------------------------------------- //
        /// Constructs box by using two vectors
        AABB(const Vec3r &v1, const Vec3r &v2);
        // ------------------------------------------------------------------------- //
        /// Checks whether v is in the AABB
        bool isInside(const Vec3r &v) const;
        // ------------------------------------------------------------------------- //
        /// Returns an optional 2D vector which contains the ray positions
        /// of the entering and the exiting positions. If there is no intersection
        /// then the optional does not contain an object.
        std::optional<Vec2r> getIntersectionPositions(const Ray &ray,
                                                      real min_t = 0.0,
                                                      real max_t = std::numeric_limits<real>::max()) const;
        // ------------------------------------------------------------------------- //
        /// Generates the intersection object for the entering part for rendering
        /// and shading.
        std::optional<Intersection> getIntersection(const Ray &ray,
                                                    real min_t = 0.0) const;
        // ------------------------------------------------------------------------- //
    };
    // ------------------------------------------------------------------------- //
}
//--------------------------------------------------------------------------//