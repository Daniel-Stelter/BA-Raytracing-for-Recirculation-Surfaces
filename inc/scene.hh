#pragma once

#include <optional>
#include <memory>

#include "renderable.hh"
#include "camera.hh"
#include "colors.hh"
#include "progresssaver.hh"
#include "recsurface.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    // ------------------------------------------------------------------------- //
    class Scene
    {
    private:
        // ------------------------------------------------------------------------- //
        RecSurface m_rec_surface;
        std::vector<std::unique_ptr<Renderable>> m_objects;
        Vec3r m_light_direction;
        color m_background;
        // ------------------------------------------------------------------------- //
    public:
        // ------------------------------------------------------------------------- //
        Scene(const RecSurface &rec_surface,
              const Vec3r &light_dir,
              const color &background = white);
        // ------------------------------------------------------------------------- //
        /// Returns color encoding for a given t0 value.
        color t0Color(const real &t0) const;
        // ------------------------------------------------------------------------- //
        /// Returns color encoding for a given tau value.
        color tauColor(const real &tau) const;
        // ------------------------------------------------------------------------- //
        /// Returns the recirculation surface of this scene.
        const RecSurface &getRecSurface() const { return m_rec_surface; }
        // ------------------------------------------------------------------------- //
        /// Returns the common objects of this scene.
        const std::vector<std::unique_ptr<Renderable>> &getObjects() const { return m_objects; }
        // ------------------------------------------------------------------------- //
        /// Adds a common object to this scene.
        void addObject(const Renderable &r) { m_objects.push_back(r.clone()); }
        // ------------------------------------------------------------------------- //
        /// Returns the direction of the directed light in this scene.
        const Vec3r &getLightDirection() const { return m_light_direction; }
        // ------------------------------------------------------------------------- //
        /// Returns the color of the background of this scene.
        const color &getBackgroundColor() const { return m_background; }
        // ------------------------------------------------------------------------- //
        /// Returns the first intersection of the ray with a common object in the scene.
        std::optional<Intersection> getCommonObjectIntersection(const Ray &ray,
                                                                real begin_at = 0.0) const;
        // ------------------------------------------------------------------------- //
        /// Raytracing through the scene along the defined ray. Can be limited by
        /// begin and end position. This does not consider the recirculation surface
        /// but only common objects.
        color raytracingCommonObjects(const Ray &ray, real begin_at = 0.0);
        // ------------------------------------------------------------------------- //
        /// Raytracing through the scene along the defined ray. Can be limited by
        /// begin and end position.
        /// rsi_result contains information whether there was a RecPoint and where
        /// it is located on the ray.
        std::array<color, 2> raytracing(const Ray &ray,
                                        RSIntersection &rsi_result,
                                        real begin_at = 0.0,
                                        real end_at = std::numeric_limits<real>::max());
        // ------------------------------------------------------------------------- //
        /// Raytracing through the scene along the defined ray. Can be limited by
        /// begin and end position.
        /// rsi_result contains information whether there was a RecPoint and where
        /// it is located on the ray.
        /// In addition, the user can get information whether the domain of the
        /// recirculation surface object was intersected (and therefore checked)
        std::array<color, 2> raytracing(const Ray &ray,
                                        RSIntersection &rsi_result,
                                        bool &rs_domain_intersected,
                                        real begin_at = 0.0,
                                        real end_at = std::numeric_limits<real>::max());
        // ------------------------------------------------------------------------- //
    };
    // ------------------------------------------------------------------------- //
}
//--------------------------------------------------------------------------//