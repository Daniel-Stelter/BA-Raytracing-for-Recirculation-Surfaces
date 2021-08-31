#pragma once

#include <memory>

#include "camera.hh"
#include "hyperline.hh"
#include "jobparams.hh"
#include "progresssaver.hh"
#include "renderable.hh"
#include "rsintersection.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    // ------------------------------------------------------------------------- //
    /// Object for handling the recirculation surface. Provides functionalities for
    /// finding intersections with rays.
    class RecSurface
    {
    private:
        // ------------------------------------------------------------------------- //
        std::shared_ptr<Flow3D> p_flow;
        DataParams m_data;
        SearchParams m_search;
        // ------------------------------------------------------------------------- //
        std::optional<RecPoint> getRecPoint(HyperLine &hl, const Ray &ray) const;
        // ------------------------------------------------------------------------- //
        bool doesLineNeedTest(const Vec3r &pA,
                              const Vec3r &pB,
                              const Camera &cam,
                              const ProgressSaver &progress,
                              const std::vector<std::unique_ptr<Renderable>> &objects) const;
        // ------------------------------------------------------------------------- //
    public:
        // ------------------------------------------------------------------------- //
        RecSurface(std::shared_ptr<Flow3D> p_flow,
                   const DataParams &data,
                   const SearchParams &search);
        // ------------------------------------------------------------------------- //
        const DataParams &getDataParams() const { return m_data; }
        const SearchParams &getSearchParams() const { return m_search; }
        // ------------------------------------------------------------------------- //
        /// Returns the ingoing and outgoing intersections of a ray with the domain.
        /// Search range can be defined.
        std::optional<Vec2r> getDomainIntersections(const Ray &ray,
                                                    real begin_at = 0.0,
                                                    real end_at = std::numeric_limits<real>::max()) const;
        // ------------------------------------------------------------------------- //
        /// Searches for a recirculation point on the given ray und returns a RSIntersection.
        /// The cam_index will be the max value and must be set manually afterwards.
        RSIntersection searchIntersection(const Ray &ray,
                                          real begin_at = 0.0,
                                          real end_at = std::numeric_limits<real>::max(),
                                          bool *needed_integration = nullptr) const;
        // ------------------------------------------------------------------------- //
        /// More complex iteration through the space by considering already calculated
        /// rays from the original scan of the space. Does only check HyperLines which
        /// are in background of already found intersections (with RecSurface or common
        /// objects). Also provides the option to calculate the HyperLines which would
        /// not be calculated (so in total, the full ray would have been calculated).
        /// Can be used for calculating light rays whith less effort.
        RSIntersection searchIntersection(const Ray &ray,
                                          const ProgressSaver &progress,
                                          const Camera &cam,
                                          const std::vector<std::unique_ptr<Renderable>> &objects,
                                          real begin_at = 0.0,
                                          real end_at = std::numeric_limits<real>::max(),
                                          bool *needed_integration = nullptr,
                                          bool invert_search = false) const;
        // ------------------------------------------------------------------------- //
        /// Estimates the normal of a given RecPoint by searching for other points in the
        /// neath. Creates multiple new Hyperlines, extracts their points and tries to
        /// approximate a normal with them.
        Vec3r estimateFlowNormal(const RecPoint &rp,
                                 const Ray &ray,
                                 real offset_space,
                                 size_t max_steps_smaller = 1) const;
        // ------------------------------------------------------------------------- //
    private:
        // ------------------------------------------------------------------------- //
        /// Helper function for normal calculation. Executes search for RecPoints inside
        /// a Hyperline and adds the 3D positions to the list if it is considered as far
        /// enough away.
        void addHyperlineToList(HyperLine &hl,
                                const RecPoint &rp,
                                std::vector<Vec3r> &points,
                                const SearchParams &sp) const;
        // ------------------------------------------------------------------------- //
        /// Helper function for normal calculation. Executes seach for RecPoints by cube
        /// layout.
        void addNeighborhoodByCube(const RecPoint &rp,
                                   std::vector<Vec3r> &points,
                                   real offset_space) const;
        // ------------------------------------------------------------------------- //
        /// Helper function for normal calculation. Executes seach for RecPoints by cross
        /// layout.
        void addNeighborhoodByCross(const RecPoint &rp,
                                    std::vector<Vec3r> &points,
                                    real offset_space) const;
        // ------------------------------------------------------------------------- //
    };
    // ------------------------------------------------------------------------- //
}
//--------------------------------------------------------------------------//