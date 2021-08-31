#pragma once

#include <optional>
#include "ray.hh"
#include "recpoint.hh"
#include "types.hh"

// --------------------------------------------------------------------------- //
namespace RS
{
    // --------------------------------------------------------------------------- //
    /// Structure for saving relevant data for a pixel for which a RecPoint
    /// is found.
    struct RSIntersection
    {
        // --------------------------------------------------------------------------- //
        size_t cam_index;           // Index of the pixel for the camera.
        Ray ray;                    // Ray of this pixel
        std::optional<real> hit;    // Position of the RecPoint on the ray.
        std::optional<RecPoint> rp; // RecPoint
        // --------------------------------------------------------------------------- //
        // constructor
        RSIntersection() = default;
        RSIntersection(size_t cam_index, Ray ray, std::optional<real> hit, std::optional<RecPoint> rp)
            : cam_index{cam_index}, ray{ray}, hit{hit}, rp{rp} {}
        // copy constructor
        RSIntersection(const RSIntersection &rsi) = default;
        // move constructor
        RSIntersection(RSIntersection &&rsi) = default;
        // copy assignment operator
        RSIntersection &operator=(const RSIntersection &rsi) = default;
        // move assignment operator
        RSIntersection &operator=(RSIntersection &&rsi) = default;
        // --------------------------------------------------------------------------- //
        bool operator==(const RSIntersection &rhs) const { return cam_index == rhs.cam_index; }
        bool operator<(const RSIntersection &rhs) const { return cam_index < rhs.cam_index; }
        bool operator>(const RSIntersection &rhs) const { return cam_index > rhs.cam_index; }
        bool operator<=(const RSIntersection &rhs) const { return cam_index <= rhs.cam_index; }
        bool operator>=(const RSIntersection &rhs) const { return cam_index >= rhs.cam_index; }
        // --------------------------------------------------------------------------- //
        /// Checks whether the t and tau values are near enough dependent on their spacial
        /// distance to be considered as neighbors.
        bool areTimeDimsCompatible(const RSIntersection &other) const
        {
            if (!rp || !other.rp)
                return false;
            const RecPoint &rp1 = rp.value(), &rp2 = other.rp.value();
            real dis = (rp1.pos - rp2.pos).norm();

            real dif_t0 = std::abs(rp1.t0 - rp2.t0);
            real dif_tau = std::abs(rp1.tau - rp2.tau);
            return dif_t0 / dis <= Globals::NEIGHBOR_DIFT0_PERLU && dif_tau / dis <= Globals::NEIGHBOR_DIFTAU_PERLU;
        }
        // --------------------------------------------------------------------------- //
        /// Checks whether the spacial positions of the RecPoints on their rays are considered
        /// as being candidates for neighbors by checking their depths and calculating an angle
        /// between the points and the "ideal" point of this object on the other's rays.
        bool areAnglesCompatible(const RSIntersection &other) const
        {
            // position on the rays nearly equally deep?
            //  -> find angle between I, pos1 and pos2 with I the point on ray2
            //     with the same depth as this RSI has ("ideal" point on ray2)
            //  -> if angle is lower than threshold, the points are considered as neighbors

            if (!rp || !other.rp)
                return false;

            const Vec3r &pos1 = rp->pos, &pos2 = other.rp->pos;
            const Vec3r dif = pos2 - pos1;

            // Test angle
            const Vec3r I = other.ray(hit.value());
            real theta = VC::math::angle(I - pos1, dif);
            return theta <= Globals::NEIGHBOR_SPACEANGLE;
        }
        // --------------------------------------------------------------------------- //
        /// Determines whether the two intersections are consideres as neighbors depending
        /// on the time values and the angle.
        bool isNeighboring(const RSIntersection &other) const
        {
            return areTimeDimsCompatible(other) && areAnglesCompatible(other);
        }
        //--------------------------------------------------------------------------//
    };
    // --------------------------------------------------------------------------- //
}
//--------------------------------------------------------------------------//