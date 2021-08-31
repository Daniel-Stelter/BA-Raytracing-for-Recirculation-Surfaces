#pragma once

#include <vector>

#include "directionallight.hh"
#include "phong.hh"
#include "raytracer.hh"

// -------------------------------------------------------------------------- //
namespace RS
{
    // -------------------------------------------------------------------------- //
    enum NormalCalcStrategy {
        NEIGHBORS,
        SAMPLING,
        HYBRID,
        NONE
    };
    // -------------------------------------------------------------------------- //
    class Shader
    {
    private:
        // -------------------------------------------------------------------------- //
        std::shared_ptr<Raytracer> m_raytracer;
        size_t m_cam_width, m_cam_height;
        const std::string m_save_dir;

        std::vector<Vec3r> m_normals;  // in fact, does only save normals for RecSurface
        std::vector<bool> m_in_shadow; // does contain info for both RecSurface and common objects
        bool m_is_normals_ready;
        bool m_is_shadows_ready;
        bool m_is_shadows_sharp;

        Phong m_phong_t0, m_phong_tau;
        DirectionalLight m_light;
        color m_back_col;

        NormalCalcStrategy m_current_normal_strategy;
        // -------------------------------------------------------------------------- //
    public:
        // -------------------------------------------------------------------------- //
        Shader(std::shared_ptr<Raytracer> raytracer,
               std::string save_dir,
               color background = white,
               double intensity = 1);
        // -------------------------------------------------------------------------- //
        /// Calculates textures depending on the currently loaded values. Does either
        /// need to use shading or shadows. Else no calculation will be executed. Also,
        /// the execution fails if at least one dependeny (normals or shadows) is not
        /// available. Their calculation or loading from disc must be started manually. 
        void createTextures(bool do_shading, bool do_shadows) const;
        // -------------------------------------------------------------------------- //
        /// Executes the calculation of normals for each found RecPoint by using the
        /// given strategy. Therefore, normals of common objects are not calculated.
        /// The calculation does only start if the provided strategy is not the same
        /// as of the loaded normals (if there are any). Providing type NONE resets
        /// current normals. 
        void calcNormals(NormalCalcStrategy strategy);
        // -------------------------------------------------------------------------- //
        /// Calculates the shadows for each found intersection of the scene (RecSurface
        /// and common objects). Does only start calculation if the shadows are currently
        /// not available.
        void calcShadows();
        // -------------------------------------------------------------------------- //
        /// Retests the rims of the shadows by recalculation each part of the light
        /// ray which was not calculated in the calcShadows() call. Asserts that the 
        /// shadow information is loaded (set by calcShadows() or loadShadows()).
        void sharpenShadows();
        // -------------------------------------------------------------------------- //
        /// Tries to load saved normals calculated by neighbors from the disc. Fails if 
        /// there is no file or the file contains too less or to many vectors.
        bool loadNormals(NormalCalcStrategy strategy);
        // -------------------------------------------------------------------------- //
        /// Tries to load saved shadow information from the disc. Fails if there is no
        /// file or the file contains too less or to many bool values.
        bool loadShadows();
        // -------------------------------------------------------------------------- //
    private:
        // -------------------------------------------------------------------------- //
        /// Saves the current vector of normals to the disc.
        void saveNormals(NormalCalcStrategy strategy) const;
        // -------------------------------------------------------------------------- //
        /// Saves the current vector of shadow information to the disc.
        void saveShadows() const;
        // -------------------------------------------------------------------------- //
        /// Returns the save location of the normals for a specific type.
        std::string getNormalSaveLocation(NormalCalcStrategy strategy) const;
        // -------------------------------------------------------------------------- //
        /// Returns the save location of the t0 texture for a specific type of normals.
        std::string getT0SaveLocation(NormalCalcStrategy strategy, bool shadow_on) const;
        // -------------------------------------------------------------------------- //
        /// Returns the save location of the tau texture for a specific type of normals.
        std::string getTauSaveLocation(NormalCalcStrategy strategy, bool shadow_on) const;
        // -------------------------------------------------------------------------- //
        /// Checks all objects of the scene (inclusive the RecSurface) for intersections
        /// with the light ray.
        bool isInShadow(const Vec3r &point) const;
        // -------------------------------------------------------------------------- //
        /// Estimates the normal by the neighboring pixels. Needs to have at least two
        /// touching neighbors with 5D-neighboring RecPoints to estimate the normal.
        Vec3r estimateNormalFromNeighbors(size_t cam_index) const;
        // -------------------------------------------------------------------------- //
        /// Returns the colors for t0 and tau. If the normal has not been calculated
        /// before, this also will be done.
        std::array<color, 2> shadeRecSurface(size_t cam_index, bool do_shading, bool do_shadows) const;
        // -------------------------------------------------------------------------- //
        /// Finds the nearest common object of the scene and shades it. For shadow
        /// calculation the RecSurface is also taken into account.
        color shadeCommonObjects(size_t cam_index, bool do_shading, bool do_shadows) const;
        // -------------------------------------------------------------------------- //
    };
    // -------------------------------------------------------------------------- //
}
// -------------------------------------------------------------------------- //