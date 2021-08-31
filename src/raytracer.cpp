#include "raytracer.hh"

#include "box.hh"
#include "shader.hh"
#include "timer.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  /// Setup of overview variables.
  void Raytracer::setOverviewVariables(size_t &checked_rays, size_t &total_rays) const
  {
    size_t width = m_cam->plane_width();
    size_t height = m_cam->plane_height();
    checked_rays = total_rays = 0;
    for (size_t index = 0; index < width * height; ++index)
    {
      size_t x = index % width, y = index / width;
      // only consider rays which intersect the domain
      if (m_scene->getRecSurface().getDomainIntersections(m_cam->ray(x, y)).has_value())
      {
        ++total_rays;
        // was the ray finished in an earlier execution?
        if (m_progress.getStartIndex() > index)
          ++checked_rays;
      }
    }
  }

  //--------------------------------------------------------------------------//
  void Raytracer::saveToDisc()
  {
    m_progress.saveData();
    m_texture_t0.write_ppm(m_save_dir + "/t0.ppm");
    m_texture_tau.write_ppm(m_save_dir + "/tau.ppm");
  }

  //--------------------------------------------------------------------------//
  void Raytracer::render()
  {
    // recreate image from progress saver
    preRenderFromProgress();

    size_t width = m_cam->plane_width();
    size_t height = m_cam->plane_height();

    // overview how many rays have to be tested in total
    size_t checked_domain_rays = 0;
    size_t total_domain_rays = 0;
    setOverviewVariables(checked_domain_rays, total_domain_rays);
    cout << "Rays with domain intersection: " << total_domain_rays << " / " << width * height << endl;
    if (checked_domain_rays > 0)
      cout << "Rays saved from past calculation: " << checked_domain_rays << endl;

    omp_lock_t lck;
    omp_init_lock(&lck);
    TimerHandler::reset();
#pragma omp parallel for schedule(dynamic)
    for (size_t cam_index = m_progress.getStartIndex(); cam_index < width * height; ++cam_index)
    {
      size_t tid = TimerHandler::overall_timer().createTimer();
      size_t x = cam_index % width;
      size_t y = cam_index / width;
      Ray ray = m_cam->ray(x, y);

      // contains the important information about whether there is a RecPoint and where
      RSIntersection rsi{cam_index, ray, {}, {}};
      // actual calculation
      bool rs_domain_intersected;
      array<color, 2> colors = m_scene->raytracing(ray,
                                                   rsi,
                                                   rs_domain_intersected);
      m_texture_t0.pixel(x, y) = colors[0];
      m_texture_tau.pixel(x, y) = colors[1];

      omp_set_lock(&lck);
      // insert result
      m_progress.update(rsi);
      // if needed: increase overview variable + save to disc + update output
      if (rs_domain_intersected)
      {
        if (++checked_domain_rays % 120 == 0)
          saveToDisc();
        // output to console
        cout << "\rFinished: " << checked_domain_rays << " / " << total_domain_rays
             << " | RecPoints found: " << m_progress.numPointsFound() << flush;
      }
      omp_unset_lock(&lck);

      TimerHandler::overall_timer().deleteTimer(tid);
    }
    saveToDisc();

    cout << "\r\33[KTotal RecPoints found: " << m_progress.numPointsFound() << " / " << total_domain_rays << endl;
  }

  //--------------------------------------------------------------------------//
  void Raytracer::preRenderFromProgress()
  {
    auto &saved_points = m_progress.getSavedPoints();
    size_t width = m_cam->plane_width();

    size_t vec_pos = 0;
    // go through all points which are saved. Ignore the rest
    for (size_t index = 0; index < m_progress.getStartIndex(); ++index)
    {
      size_t x = index % width, y = index / width;
      if (saved_points.size() > vec_pos && index == saved_points[vec_pos].cam_index)
      {
        // recreate pixel color by color mapping
        m_texture_t0.pixel(x, y) = m_scene->t0Color(saved_points[vec_pos].rp.value().t0);
        m_texture_tau.pixel(x, y) = m_scene->tauColor(saved_points[vec_pos].rp.value().tau);
        ++vec_pos;
      }
      else
        m_texture_t0.pixel(x, y) = m_texture_tau.pixel(x, y) =
            m_scene->raytracingCommonObjects(m_cam->ray(x, y));
    }
  }

  //--------------------------------------------------------------------------//
  void Raytracer::renderSpace()
  {
    size_t width = (size_t)m_cam->plane_width();
    size_t height = (size_t)m_cam->plane_height();
    Texture t{width, height};

    Box box{m_scene->getRecSurface().getDataParams().domain};
    for (size_t y = 0; y < height; ++y)
    {
      for (size_t x = 0; x < width; ++x)
      {
        Ray ray = m_cam->ray(x, y);
        DirectionalLight light{m_scene->getLightDirection()};
        auto hit = box.getIntersection(ray);
        t.pixel(x, y) = hit.has_value() ? box.shade(light, hit.value()) : m_scene->getBackgroundColor();
      }
    }
    t.write_ppm(m_save_dir + "/space.ppm");
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//