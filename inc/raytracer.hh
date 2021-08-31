#pragma once

#include <omp.h> // parallelization
#include <vector>

#include "camera.hh"
#include "progresssaver.hh"
#include "scene.hh"
#include "texture.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  /// Renders a scene viewed from a camera into an image
  class Raytracer
  {
  protected:
    //--------------------------------------------------------------------------//
    std::shared_ptr<Camera> m_cam;
    std::shared_ptr<Scene> m_scene;
    ProgressSaver m_progress;
    Texture m_texture_t0, m_texture_tau;
    std::string m_save_dir;
    //--------------------------------------------------------------------------//
    /// Function which looks up how many rays have to be applied to the RecSurface.
    /// Also consideres how many ray results have already been saved.
    virtual void setOverviewVariables(size_t &checked_rays, size_t &total_rays) const;
    //--------------------------------------------------------------------------//
    /// Using the progress saver, recreates all already calculated pixel and fills textures.
    virtual void preRenderFromProgress();
    //--------------------------------------------------------------------------//
    /// Save all unsaved data from the progress saver and stores both textures.
    virtual void saveToDisc();
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    Raytracer(std::shared_ptr<Camera> cam,
              std::shared_ptr<Scene> scene,
              const std::string &save_dir)
        : m_cam{cam},
          m_scene{scene},
          m_progress{save_dir, cam->plane_width(), cam->plane_height()},
          m_texture_t0{cam->plane_width(), cam->plane_height()},
          m_texture_tau{cam->plane_width(), cam->plane_height()},
          m_save_dir{save_dir}
    {
      m_progress.loadData(*cam);
    }
    //--------------------------------------------------------------------------//
    virtual ~Raytracer() = default;
    //--------------------------------------------------------------------------//
    /// Returns the camera.
    const std::shared_ptr<Camera> &getCamera() const { return m_cam; }
    //--------------------------------------------------------------------------//
    /// Returns the scene.
    const std::shared_ptr<Scene> &getScene() const { return m_scene; }
    //--------------------------------------------------------------------------//
    /// Returns the progress saver.
    const ProgressSaver &getProgress() const { return m_progress; }
    //--------------------------------------------------------------------------//
    /// Returns the texture for t0.
    const Texture &getTextureT0() const { return m_texture_t0; }
    //--------------------------------------------------------------------------//
    /// Returns the texture for tau.
    const Texture &getTextureTau() const { return m_texture_tau; }
    //--------------------------------------------------------------------------//
    /// Returns the current save directory.
    const std::string &getSaveDir() const { return m_save_dir; }
    /// Returns the current save directory.
    std::string &getSaveDir() { return m_save_dir; }
    //--------------------------------------------------------------------------//
    /// Executes the calculation of the RecSurface and other objects in the scene
    /// without shading them. This can be done after this calculation by the
    /// renderShaded method.
    virtual void render();
    //--------------------------------------------------------------------------//
    /// Renders a simple representation of the RecSurface domain which will be used.
    virtual void renderSpace();
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//