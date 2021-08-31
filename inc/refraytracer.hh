#pragma once

#include "raytracer.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    /// Used to generate higher resolutions of images created with another raytracer.
    /// Needs to get a camera with same settings except higher resolution.
    class RefinementRaytracer : public Raytracer
    {
    private:
        //--------------------------------------------------------------------------//
        const size_t m_res_increase; // Multiplier of the resolution of the "parent" calculation.
        const ProgressSaver &m_old_progress;
        //--------------------------------------------------------------------------//
        /// Function which looks up how many rays have to be applied to the RecSurface.
        /// Also consideres how many ray results have already been saved.
        void setOverviewVariables(size_t &checked_rays, size_t &total_rays) const override;
        //--------------------------------------------------------------------------//
        /// Checks whether the result of the old progress saver can be adopted because
        /// of using the exact same ray.
        bool canRayBeAdopted(size_t x, size_t y) const;
        //--------------------------------------------------------------------------//
        /// Finds the nearest intersection of all near pixel of the old raytracer with the RecSurface.
        std::optional<real> getNearestIntersection(size_t x, size_t y) const;
        //--------------------------------------------------------------------------//
        /// Finds the nearest intersection of all near pixel of the old raytracer with the RecSurface.
        std::optional<real> getNearestIntersection(size_t cam_index) const;
        //--------------------------------------------------------------------------//
    public:
        //--------------------------------------------------------------------------//
        RefinementRaytracer(const Raytracer *basic_rt,
                            size_t res_increase,
                            const std::string &save_dir);
        //--------------------------------------------------------------------------//
        virtual ~RefinementRaytracer() = default;
        //--------------------------------------------------------------------------//
        /// Executes the calculation of the RecSurface and other objects in the scene
        /// without shading them. This can be done after this calculation by the
        /// renderShaded method.
        virtual void render() override;
        //--------------------------------------------------------------------------//
        /// Searches for edges and tests all rays of points in background of these edges from
        /// completely new. Iterates till no new points in foreground are found.
        virtual void postProcessing();
        //--------------------------------------------------------------------------//
    };
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//