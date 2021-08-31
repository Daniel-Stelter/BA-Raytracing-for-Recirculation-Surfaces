#pragma once

#include <fstream> // file io
#include <vector>
#include "camera.hh"
#include "rsintersection.hh"

// --------------------------------------------------------------------------- //
namespace RS
{
    // --------------------------------------------------------------------------- //
    /// Saves the progress of the calculation of the texture automatically.
    /// Therefore, the calculation can be interrupted.
    /// StartPosition indicates at which the program has to continue with the normal
    /// search for futher RecPoints.
    /// Attention: Works only if the points are inserted in approximately correct order.
    class ProgressSaver
    {
    private:
        // --------------------------------------------------------------------------- //
        size_t start_index;                  // indicates the ray with which the raytracer needs to start next time
        std::vector<RSIntersection> saved;   // contains data for found RecPoints
        std::vector<RSIntersection> waiting; // contains all out-of-order updates (positives and negatives) in correct order

        bool complete_rewrite;
        size_t next_save_index;                 // index for "saved" indicating which points still need to be written to the file
        size_t count_waiting_positives;         // indicates how many points in "waiting" are RecPoints
        const std::string file_start, file_vec; // location / name of the save files

        size_t width, height;
        size_t **index_map;
        // --------------------------------------------------------------------------- //
    public:
        // --------------------------------------------------------------------------- //
        ProgressSaver(const std::string &save_dir, size_t cam_width, size_t cam_height);
        // --------------------------------------------------------------------------- //
        ~ProgressSaver();
        // --------------------------------------------------------------------------- //
        size_t getStartIndex() const { return start_index; }
        // --------------------------------------------------------------------------- //
        const std::vector<RSIntersection> &getSavedPoints() const { return saved; }
        // --------------------------------------------------------------------------- //
        const RSIntersection *getRSI(size_t x, size_t y) const
        {
            if (x >= width || y >= height || index_map[x][y] >= saved.size())
                return nullptr;
            return &saved[index_map[x][y]];
        }
        const RSIntersection *getRSI(size_t cam_index) const { return getRSI(cam_index % width, cam_index / width); }
        // --------------------------------------------------------------------------- //
        size_t getRSIPosition(size_t x, size_t y) const
        {
            if (x >= width || y >= height)
                return std::numeric_limits<size_t>::max();
            return index_map[x][y];
        }
        size_t getRSIPosition(size_t cam_index) const { return getRSIPosition(cam_index % width, cam_index / width); }
        // --------------------------------------------------------------------------- //
        size_t numPointsFound() const { return saved.size() + count_waiting_positives; }
        // --------------------------------------------------------------------------- //
        void update(const RSIntersection &data);
        void saveData();
        void loadData(const Camera &cam);
        // --------------------------------------------------------------------------- //
    };
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//