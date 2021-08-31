#include "refraytracer.hh"

#include "timer.hh"

using namespace std;

namespace RS
{
  //--------------------------------------------------------------------------//
  RefinementRaytracer::RefinementRaytracer(const Raytracer *basic_rt,
                                           size_t res_increase,
                                           const string &save_dir)
      : Raytracer{basic_rt->getCamera()->create_increased(res_increase), basic_rt->getScene(), save_dir},
        m_res_increase{res_increase},
        m_old_progress{basic_rt->getProgress()} {}

  //--------------------------------------------------------------------------//
  /// Setup of overview variables.
  void RefinementRaytracer::setOverviewVariables(size_t &checked_rays, size_t &total_rays) const
  {
    size_t width = m_cam->plane_width();
    size_t height = m_cam->plane_height();
    checked_rays = total_rays = 0;
    size_t adopted_rays = 0;
    for (size_t index = 0; index < width * height; ++index)
    {
      size_t x = index % width, y = index / width;
      // can it be copied?
      if (canRayBeAdopted(x, y))
      {
        if (m_old_progress.getRSI(x, y))
          ++adopted_rays;
      }
      else
      {
        // was a point found by old raytracer and does it hit the domain?
        if (getNearestIntersection(index).has_value() &&
            m_scene->getRecSurface().getDomainIntersections(m_cam->ray(x, y)).has_value())
        {
          ++total_rays;
          // already finished?
          if (index < m_progress.getStartIndex())
            ++checked_rays;
        }
      }
    }
    if (adopted_rays > 0)
      cout << "Adopted rays by old raytracer (not included in total rays): " << adopted_rays << endl;
  }

  //--------------------------------------------------------------------------//
  bool RefinementRaytracer::canRayBeAdopted(size_t x, size_t y) const
  {
    // is the new pixel in the middle of the old one?
    return m_res_increase % 2 == 1 &&
           x % m_res_increase == m_res_increase / 2 &&
           y % m_res_increase == m_res_increase / 2;
  }

  //--------------------------------------------------------------------------//
  optional<real> RefinementRaytracer::getNearestIntersection(size_t x, size_t y) const
  {
    assert(x < m_cam->plane_width());
    assert(y < m_cam->plane_height());
    x /= m_res_increase;
    y /= m_res_increase;
    real nearest = numeric_limits<real>::max();

    // consider old corresponding pixel and its neighbors
    const RSIntersection *neighbor_RSIs[5] = {
        m_old_progress.getRSI(x, y),
        m_old_progress.getRSI(x - 1, y),
        m_old_progress.getRSI(x + 1, y),
        m_old_progress.getRSI(x, y - 1),
        m_old_progress.getRSI(x, y + 1)};

    for (auto n : neighbor_RSIs)
      if (n && n->hit.value() < nearest) // check if RecPoint and if it is new nearest
        nearest = n->hit.value();

    // if no intersection was found: empty optional
    if (nearest == numeric_limits<real>::max())
      return {};
    return nearest;
  }

  //--------------------------------------------------------------------------//
  optional<real> RefinementRaytracer::getNearestIntersection(size_t cam_index) const
  {
    return getNearestIntersection(cam_index % m_cam->plane_width(), cam_index / m_cam->plane_width());
  }

  //--------------------------------------------------------------------------//
  void RefinementRaytracer::render()
  {
    preRenderFromProgress();

    size_t width = m_cam->plane_width();
    size_t height = m_cam->plane_height();

    // overview how many rays have to be tested in total
    size_t checked_domain_rays = 0;
    size_t total_domain_rays = 0;
    setOverviewVariables(checked_domain_rays, total_domain_rays);
    cout << "Rays considering for search: " << total_domain_rays << " / " << width * height << endl;
    if (checked_domain_rays > 0)
      cout << "Rays saved from last calculation: " << checked_domain_rays << endl;

    omp_lock_t lck;
    omp_init_lock(&lck);

    TimerHandler::reset();
#pragma omp parallel for schedule(dynamic)
    for (size_t cam_index = m_progress.getStartIndex(); cam_index < width * height; ++cam_index)
    {
      // start overall timer
      size_t tid = TimerHandler::overall_timer().createTimer();

      size_t x = cam_index % width, y = cam_index / width;
      Ray ray = m_cam->ray(x, y);
      RSIntersection rsi{cam_index, ray, {}, {}};

      bool needs_test, rs_domain_intersected = false;
      array<color, 2> colors;

      // special case: take over value of old raytracer
      if (canRayBeAdopted(x, y))
      {
        needs_test = false;
        auto old_rsi = m_old_progress.getRSI(x / m_res_increase, y / m_res_increase);
        if (old_rsi)
        {
          rsi.hit = old_rsi->hit;
          rsi.rp = old_rsi->rp;
          colors = {m_scene->t0Color(rsi.rp->t0), m_scene->tauColor(rsi.rp->tau)};
        }
        else
        {
          color c = m_scene->raytracingCommonObjects(ray);
          colors = {c, c};
        }
      }
      else
      {
        // find out start position
        auto nearest = getNearestIntersection(x, y);
        needs_test = nearest.has_value();

        if (needs_test)
          colors = m_scene->raytracing(ray,
                                       rsi,
                                       rs_domain_intersected,
                                       nearest.value() - Globals::RAYBACKOFFSET_REFINEMENT); // set back
        else
        {
          color c = m_scene->raytracingCommonObjects(ray);
          colors = {c, c};
        }
      }
      m_texture_t0.pixel(x, y) = colors[0];
      m_texture_tau.pixel(x, y) = colors[1];

      omp_set_lock(&lck);
      // save rsi
      m_progress.update(rsi);
      if (rs_domain_intersected)
        ++checked_domain_rays;
      if (needs_test)
      {
        // save in regular intervals
        if (checked_domain_rays % 1000 == 0)
          saveToDisc();
        cout << "\rFinished: " << checked_domain_rays << " / " << total_domain_rays
             << " | RecPoints found: " << m_progress.numPointsFound() << flush;
      }
      omp_unset_lock(&lck);
      // end overall timer
      TimerHandler::overall_timer().deleteTimer(tid);
    }
    m_progress.saveData();
    m_texture_t0.write_ppm(m_save_dir + "/t0.ppm");
    m_texture_tau.write_ppm(m_save_dir + "/tau.ppm");
    cout << "\r\33[KTotal RecPoints found: " << m_progress.numPointsFound() << " / " << total_domain_rays << endl;
  }

  //--------------------------------------------------------------------------//
  void RefinementRaytracer::postProcessing()
  {
    size_t width = m_cam->plane_width(), height = m_cam->plane_height();

    // 1. Create 2D array containing information about which rays are completely tested (default: false)
    bool **completely_tested = new bool *[width];
    for (size_t x = 0; x < width; ++x)
    {
      completely_tested[x] = new bool[height];
      for (size_t y = 0; y < height; ++y)
      {
        completely_tested[x][y] = false;
      }
    }

    // 2. Test all points which are on an edge in 5D
    bool new_test = true;
    size_t iteration = 1, rays_tested = 0, new_found = 0, new_found_total = 0, old_total = m_progress.numPointsFound();
    omp_lock_t lck;
    omp_init_lock(&lck);

    TimerHandler::reset();
    while (new_test)
    {
      new_test = false;
      cout << "\rIteration " << iteration << " | RecPoints found: 0 / 0" << flush;
#pragma omp parallel for schedule(dynamic)
      for (size_t cam_index = 0; cam_index < width * height; ++cam_index)
      {
        // start overall timer
        size_t tid = TimerHandler::overall_timer().createTimer();
        size_t x = cam_index % width, y = cam_index / width;

        // completely tested rays need no further calculation
        if (completely_tested[x][y])
        {
          TimerHandler::overall_timer().deleteTimer(tid);
          continue;
        }

        const RSIntersection *rsi = m_progress.getRSI(x, y);
        // look at all four neighbors of new sampling
        const RSIntersection *neighbor_RSIs[4] = {
            m_progress.getRSI(x, y - 1),
            m_progress.getRSI(x - 1, y),
            m_progress.getRSI(x, y + 1),
            m_progress.getRSI(x + 1, y)};

        // no neighbors means there is no test needed
        if (!neighbor_RSIs[0] && !neighbor_RSIs[1] && !neighbor_RSIs[2] && !neighbor_RSIs[3])
        {
          TimerHandler::overall_timer().deleteTimer(tid);
          continue;
        }

        // otherwise check case: has own ray an intersection?
        if (rsi)
        {
          bool needs_test = false;
          for (auto neighbor : neighbor_RSIs)
          {
            // is there any neighbor which is nearer to the cam and no 5D neighbor?
            if (neighbor &&
                neighbor < rsi &&
                !rsi->isNeighboring(*neighbor))
            {
              // then test!
              needs_test = true;
              break;
            }
          }
          // if no such neighbor: no test needed
          if (!needs_test)
          {
            TimerHandler::overall_timer().deleteTimer(tid);
            continue;
          }
        }

        // now it is clear that there will be a test
        completely_tested[x][y] = true;

        Ray ray = m_cam->ray(x, y);
        RSIntersection test_result{cam_index, ray, {}, {}};

        // search for the position on the ray where the original search began (if there was one)
        auto nearest = getNearestIntersection(x, y);
        real old_start = nearest.has_value()
                             ? nearest.value() - Globals::RAYBACKOFFSET_REFINEMENT
                             : numeric_limits<real>::max();

        bool rs_domain_intersected = false;
        array<color, 2> colors = m_scene->raytracing(ray,
                                                     test_result,
                                                     rs_domain_intersected,
                                                     0.0,
                                                     old_start); // test only up to the found point
        // if there is a new point: change pixel colors
        if (test_result.rp)
        {
          m_texture_t0.pixel(x, y) = colors[0];
          m_texture_tau.pixel(x, y) = colors[1];
        }

        omp_set_lock(&lck);
        if (rs_domain_intersected)
          ++rays_tested;
        if (test_result.rp)
        {
          ++new_found;
          m_progress.update(test_result);
          new_test = true;
        }
        cout << "\rIteration " << iteration << " | RecPoints found: " << new_found << " / " << rays_tested << flush;
        omp_unset_lock(&lck);
        TimerHandler::overall_timer().deleteTimer(tid);
      }
      // save files
      m_progress.saveData();
      m_texture_t0.write_ppm(m_save_dir + "/t0_postpr.ppm");
      m_texture_tau.write_ppm(m_save_dir + "/tau_postpr.ppm");
      // prepare next iteration
      ++iteration;
      new_found_total += new_found;
      rays_tested = new_found = 0;
      cout << endl;
    }
    size_t dif_new_old = m_progress.numPointsFound() - old_total;
    cout << "Total RecPoints found: " << new_found_total
         << " (new: " << dif_new_old
         << ", updated: " << new_found_total - dif_new_old << ")" << endl;
    // delete created arrays
    for (size_t x = 0; x < width; ++x)
      delete[] completely_tested[x];
    delete[] completely_tested;
  }
  //--------------------------------------------------------------------------//
}