#pragma once
#include <vector>

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    /// Defines a 2D line from p1 to p2 which are also its boundaries.
    struct Line2D
    {
        //--------------------------------------------------------------------------//
        Vec2r p1, p2;
        //--------------------------------------------------------------------------//
        Line2D(const Vec2r &p1, const Vec2r &p2) : p1{p1}, p2{p2} {}
        //--------------------------------------------------------------------------//
        /// Returns the position on the line where the nearest point to v is located
        real getNearestLinePos(const Vec2r &v)
        {
            Vec2r line_dir = p2 - p1;
            real line_len = line_dir.norm();
            if (line_len < Globals::ZERO)
                return 0.5;
            Vec2r point_dir = v - p1;
            real d = (line_dir / line_len | point_dir) / line_len;
            return std::min(std::max(d, 0.0), 1.0);
        }
        //--------------------------------------------------------------------------//
        /// Returns the neares point of the line to v
        Vec2r getNearestLinePoint(const Vec2r &v)
        {
            Vec2r line_dir = p2 - p1;
            return p1 + line_dir * getNearestLinePos(v);
        }
        //--------------------------------------------------------------------------//
        /// Finds the nearest point of the line to v and returns its distance
        real getNearestDistance(const Vec2r &v)
        {
            Vec2r nearest = getNearestLinePoint(v);
            return (v - nearest).norm();
        }
        //--------------------------------------------------------------------------//
        /// Returns all discretizised points of the line, where the thickness
        /// defines the radius at which a point is accepted.
        /// Default value is approx the square root of 0.5 so that all intersected
        /// pixel are accepted.
        std::vector<Vec2i> getLinePoints(real thickness = 0.7072)
        {
            std::vector<Vec2i> points;
            // min and max reachable pixels with the given thickness
            Vec2i v_min{(int)(std::min(p1[0], p2[0]) - thickness), (int)(std::min(p1[1], p2[1]) - thickness)};
            Vec2i v_max{(int)(std::max(p1[0], p2[0]) + thickness), (int)(std::max(p1[1], p2[1]) + thickness)};
            for (int y = v_min[1]; y <= v_max[1]; ++y)
                for (int x = v_min[0]; x <= v_max[0]; ++x)
                    if (getNearestDistance(Vec2r{(real)x, (real)y}) <= thickness)
                        points.push_back(Vec2i{x, y});
            return points;
        }
        //--------------------------------------------------------------------------//
    };
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//