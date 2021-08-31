#pragma once

#include <filesystem>
#include <memory>

#include "amiradataset.hh"
#include "box.hh"
#include "perspectivecamera.hh"
#include "doublegyre3D.hh"
#include "scene.hh"

namespace RS
{
    /// Abstract class for setting up a scene with a recirculation surface
    class SceneSetup
    {
    protected:
        std::shared_ptr<Scene> m_scene;
        SceneSetup() : m_scene{nullptr} {}

    public:
        /// Creates a camera with internally specified camera settings.
        /// Resolution can be multiplied by parametre;
        virtual std::shared_ptr<Camera> create_cam(size_t res_multiplier = 1) const = 0;
        const std::shared_ptr<Scene> &get_scene() const { return m_scene; };
    };

    class SetupDoubleGyre3D : public SceneSetup
    {
    public:
        SetupDoubleGyre3D(real ray_step_size = 0.01, real time_step_size = 0.2)
        {
            Vec3r dMin(0.01, 0.01, 0.01), dMax(1.99, 0.99, 0.99);
            DataParams data(dMin, dMax, ray_step_size);
            SearchParams search(0., 10., 0., 10., time_step_size, Globals::SEARCHPREC);

            std::shared_ptr<Flow3D> flow = std::make_shared<DoubleGyre3D>();
            RecSurface rec_surface{flow, data, search};

            Vec3r light_dir{0, -0.2, -1.0};

            m_scene.reset(new Scene(rec_surface, light_dir));
            m_scene->addObject(Box{AABB{{0, 0, 0}, {2, 1, -0.1}}});
        }

        virtual std::shared_ptr<Camera>
        create_cam(size_t res_multiplier = 1) const override
        {
            Vec3r eye{1, -1, 1.9}, look_at{1, 2, -1};
            return std::make_shared<PerspectiveCamera>(eye, look_at, 70, res_multiplier * 150, res_multiplier * 50, CamUp::Z);
        }
    };

    class SetupSquaredCylinder : public SceneSetup
    {
    public:
        SetupSquaredCylinder(real ray_step_size = 0.0025, real time_step_size = 0.1)
        {
            Vec3r dMin(0.5, -0.65, 0.01), dMax(2.5, 0.65, 5.99);
            DataParams data(dMin, dMax, ray_step_size);
            SearchParams search(0., 4.8, Globals::TAUMIN, 6.0, time_step_size, Globals::SEARCHPREC);

            std::string path = "../../SquareCylinderHighResTime";
            std::set<std::string> file_set; // we need this container for sorting
            for (const auto &entry : std::filesystem::directory_iterator(path))
                if (entry.is_regular_file())
                    file_set.insert(entry.path());
            std::vector<std::string> file_vec; // the amira flow needs a vector
            for (const auto &entry : file_set)
            {
                file_vec.push_back(entry);
                if (160 == file_vec.size()) // control how many files get loaded (each fo a time span of 0.08)
                    break;
            }
            Vec2r time_range(0.0, 0.08 * file_vec.size());

            std::shared_ptr<Flow3D> flow = std::make_shared<AmiraDataSet>(file_vec, time_range);
            RecSurface rec_surface{flow, data, search};

            Vec3r light_dir{-0.2, -1.0, 0};

            m_scene.reset(new Scene(rec_surface, light_dir));
            m_scene->addObject(Box{AABB{{-0.8, -0.65, 0}, {0.5, 0.65, 6}}});
        }

        virtual std::shared_ptr<Camera>
        create_cam(size_t res_multiplier = 1) const override
        {
            Vec3r eye{5.3, 3, -4}, look_at{0, -0.8, 3};
            Vec3r dir = (look_at - eye).normalize();
            look_at = eye + dir;
            return std::make_shared<PerspectiveCamera>(eye, look_at, 25, res_multiplier * 300, res_multiplier * 175, CamUp::Y);
        }
    };
}