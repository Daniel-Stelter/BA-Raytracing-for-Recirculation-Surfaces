#include "shader.hh"

#include <fstream>
#include <omp.h>

#include "ray.hh"
#include "scene.hh"
#include "timer.hh"

using namespace std;

// ------------------------------------------------------------------------- //
namespace RS
{
    // ------------------------------------------------------------------------- //
    Shader::Shader(std::shared_ptr<Raytracer> raytracer,
                   std::string save_dir,
                   color background,
                   double intensity)
        : m_raytracer{raytracer},
          m_cam_width{m_raytracer->getCamera()->plane_width()},
          m_cam_height{m_raytracer->getCamera()->plane_height()},
          m_save_dir{save_dir},
          m_normals{},
          m_in_shadow{},
          m_is_normals_ready{false},
          m_is_shadows_ready{false},
          m_is_shadows_sharp{false},
          m_phong_t0{raytracer->getTextureT0(), 0.6, 0.5, 0.2, 5},
          m_phong_tau{raytracer->getTextureTau(), 0.6, 0.5, 0.2, 5},
          m_light{raytracer->getScene()->getLightDirection(), {intensity, intensity, intensity}},
          m_back_col{background},
          m_current_normal_strategy{NONE}
    {
        size_t total = m_cam_width * m_cam_height;
        // init normals and shadows with zero-vector / false
        m_normals.reserve(total);
        for (size_t i = 0; i < total; ++i)
            m_normals.push_back(Vec3r{0, 0, 0});
        m_in_shadow.reserve(total);
        for (size_t i = 0; i < total; ++i)
            m_in_shadow.push_back(false);
    }

    // ------------------------------------------------------------------------- //
    void Shader::createTextures(bool do_shading, bool do_shadows) const
    {
        // some invlaid cases
        if (!do_shading && !do_shadows)
        {
            cout << "Did not start texture calculation (at least shading or shadows must be enabled)" << endl;
            return;
        }
        if (do_shading && !m_is_normals_ready)
        {
            cout << "Did not start texture calculation (normals for shading not available)" << endl;
            return;
        }
        if (do_shadows && !m_is_shadows_ready)
        {
            cout << "Did not start texture calculation (shadows not available)" << endl;
            return;
        }

        Texture texture_t0{m_cam_width, m_cam_height, m_back_col};
        Texture texture_tau{m_cam_width, m_cam_height, m_back_col};

#pragma omp parallel for schedule(dynamic)
        for (size_t cam_index = 0; cam_index < m_cam_width * m_cam_height; ++cam_index)
        {
            size_t x = cam_index % m_cam_width, y = cam_index / m_cam_width;

            // if there is a Rec Point, shade it, else shade common objects
            if (m_raytracer->getProgress().getRSI(x, y))
            {
                auto colors = shadeRecSurface(cam_index, do_shading, do_shadows);
                texture_t0.pixel(x, y) = colors[0];
                texture_tau.pixel(x, y) = colors[1];
            }
            else
                texture_t0.pixel(x, y) = texture_tau.pixel(x, y) = shadeCommonObjects(cam_index, do_shading, do_shadows);
        }
        NormalCalcStrategy strategy = do_shading ? m_current_normal_strategy : NONE;
        texture_t0.write_ppm(getT0SaveLocation(strategy, do_shadows));
        texture_tau.write_ppm(getTauSaveLocation(strategy, do_shadows));
    }

    // ------------------------------------------------------------------------- //
    void Shader::calcNormals(NormalCalcStrategy strategy)
    {
        // catch special cases
        if (strategy == NONE)
        {
            cout << "Reset normals" << endl;
            m_is_normals_ready = false;
            m_current_normal_strategy = NONE;
            return;
        }
        if (strategy == m_current_normal_strategy)
        {
            cout << "Did not start normal calculation (loaded type is equal)" << endl;
            return;
        }
        const ProgressSaver &progress = m_raytracer->getProgress();
        size_t num_finished = 0, num_successfull = 0;
        // start parallel execution
        omp_lock_t lck{};
        omp_init_lock(&lck);
        TimerHandler::reset();
#pragma omp parallel for schedule(dynamic)
        for (size_t cam_index = 0; cam_index < m_cam_width * m_cam_height; ++cam_index)
        {
            // start overall timer
            size_t tid = TimerHandler::overall_timer().createTimer();

            bool success = false;
            m_normals[cam_index] = Vec3r(0, 0, 0);

            // check if there was a RSI / RecPoint found
            const RSIntersection *rsi = progress.getRSI(cam_index);
            if (rsi)
            {
                Vec3r &n = m_normals[cam_index];
                // NEIGHBORS or HYBRID
                if (strategy != SAMPLING)
                {
                    // calculate normal by neighbors and save the value
                    n = estimateNormalFromNeighbors(cam_index);
                    // check if it was successfull
                    success = n[0] != 0 || n[1] != 0 || n[2] != 0;
                }
                // SAMPLING or HYBRID and test is needed
                if (strategy != NEIGHBORS && !success)
                {
                    n = m_raytracer->getScene()->getRecSurface().estimateFlowNormal(
                        rsi->rp.value(),
                        rsi->ray,
                        Globals::NORMAL_SEARCHDIS,
                        Globals::NORMAL_MAXSTEPS);
                    success = n[0] != 0 || n[1] != 0 || n[2] != 0;
                }
                omp_set_lock(&lck);
                if (success)
                    ++num_successfull;
                if (strategy != NEIGHBORS)
                    cout << "\rFinished: " << ++num_finished << " / " << progress.numPointsFound() << flush;
                omp_unset_lock(&lck);
            }
            // close overall timer
            TimerHandler::overall_timer().deleteTimer(tid);
        }
        cout << "\r\33[KTotal normals found: " << num_successfull << " / " << progress.numPointsFound() << endl;
        // set overview values
        m_is_normals_ready = true;
        m_current_normal_strategy = strategy;
        saveNormals(strategy);
    }

    // ------------------------------------------------------------------------- //
    void Shader::calcShadows()
    {
        // special case: already calculated
        if (m_is_shadows_ready)
        {
            cout << "Did not start shadow calculation (results are already given)" << endl;
            return;
        }

        const ProgressSaver &progress = m_raytracer->getProgress();
        // find out how many tests are needed for better output
        size_t num_total_tests = 0;
        for (size_t x = 0; x < m_cam_width; ++x)
            for (size_t y = 0; y < m_cam_height; ++y)
                if (progress.getRSI(x, y) ||
                    m_raytracer->getScene()->getCommonObjectIntersection(m_raytracer->getCamera()->ray(x, y)))
                    ++num_total_tests;
        size_t num_tested = 0, num_found = 0;

        // start parallel execution
        omp_lock_t lck{};
        omp_init_lock(&lck);
        TimerHandler::reset();
#pragma omp parallel for schedule(dynamic)
        for (size_t cam_index = 0; cam_index < m_cam_width * m_cam_height; ++cam_index)
        {
            size_t tid = TimerHandler::overall_timer().createTimer();

            size_t x = cam_index % m_cam_width, y = cam_index / m_cam_width;
            // find the position in 3D which has to be checked
            optional<Vec3r> pos{};
            // case 1: RecSurface is nearest object
            if (progress.getRSI(cam_index))
                pos = progress.getRSI(cam_index)->rp->pos;
            // case 2: find intersection with common objects
            else
            {
                Ray ray = m_raytracer->getCamera()->ray(x, y);
                auto opt = m_raytracer->getScene()->getCommonObjectIntersection(ray);
                if (opt)
                    pos = opt->position;
            }
            // set value
            if (pos.has_value())
            {
                bool result = isInShadow(pos.value());
                m_in_shadow[cam_index] = result;
                omp_set_lock(&lck);
                if (result)
                    ++num_found;
                cout << "\rFinished: " << ++num_tested << " / " << num_total_tests
                     << " | Shadows found: " << num_found << flush;
                omp_unset_lock(&lck);
            }
            else
                m_in_shadow[cam_index] = false;

            TimerHandler::overall_timer().deleteTimer(tid);
        }
        cout << "\r\33[KTotal shadows found: " << num_found << " / " << num_total_tests << endl;
        m_is_shadows_ready = true;
        saveShadows();
    }

    // ------------------------------------------------------------------------- //
    void Shader::sharpenShadows()
    {
        if (!m_is_shadows_ready)
        {
            cout << "Did not calculate shadow sharpening (needs basic calculation first)" << endl;
            return;
        }
        if (m_is_shadows_sharp)
        {
            cout << "Did not calculate shadow sharpening (already done)" << endl;
            return;
        }
        // save which light rays have already been tested completely (default: false)
        bool **completely_tested = new bool *[m_cam_width];
        for (size_t x = 0; x < m_cam_width; ++x)
        {
            completely_tested[x] = new bool[m_cam_height];
            for (size_t y = 0; y < m_cam_height; ++y)
                completely_tested[x][y] = false;
        }

        bool new_test = true;
        size_t iteration = 1, num_tested = 0, num_found = 0, num_found_total = 0;
        omp_lock_t lck;
        omp_init_lock(&lck);
        TimerHandler::reset();
        while (new_test)
        {
            cout << "\rIteration " << iteration << " | Shadows found: 0 / 0" << flush;
            new_test = false;
#pragma omp parallel for schedule(dynamic)
            for (size_t cam_index = 0; cam_index < m_cam_width * m_cam_height; ++cam_index)
            {
                size_t tid = TimerHandler::overall_timer().createTimer();

                size_t x = cam_index % m_cam_width, y = cam_index / m_cam_width;
                // skip if it is in shadow itself
                if (m_in_shadow[cam_index] || completely_tested[x][y])
                {
                    TimerHandler::overall_timer().deleteTimer(tid);
                    continue;
                }
                // check each neighbor if it is in shadow
                bool test = (x > 0 && m_in_shadow[cam_index - 1]) ||
                            (x < m_cam_width - 1 && m_in_shadow[cam_index + 1]) ||
                            (y > 0 && m_in_shadow[cam_index - m_cam_width]) ||
                            (y < m_cam_height - 1 && m_in_shadow[cam_index + m_cam_height]);
                if (!test)
                {
                    TimerHandler::overall_timer().deleteTimer(tid);
                    continue;
                }
                // find out position
                Vec3r pos;
                // might be from RecSurface...
                const RSIntersection *temp_rsi = m_raytracer->getProgress().getRSI(x, y);
                if (temp_rsi)
                    pos = temp_rsi->rp->pos;
                // ... or from a common object
                else
                {
                    auto opt = m_raytracer->getScene()->getCommonObjectIntersection(
                        m_raytracer->getCamera()->ray(x, y));
                    // if neither the one thing nor the other: skip
                    if (!opt)
                    {
                        TimerHandler::overall_timer().deleteTimer(tid);
                        continue;
                    }
                    pos = opt->position;
                }
                // common objects don't need to be retested since they were completely tested
                Ray light_ray{pos, -m_light.light_direction_to(pos)};
                // now test the parts of the light ray not tested before
                RSIntersection rsi = m_raytracer->getScene()->getRecSurface().searchIntersection(
                    light_ray,
                    m_raytracer->getProgress(),
                    *(m_raytracer->getCamera()),
                    m_raytracer->getScene()->getObjects(),
                    Globals::RAYFOREOFFSET_SHADOWS, // sets start pos on ray -> no self intersection
                    numeric_limits<real>::max(),    // default
                    nullptr,                        // default / unnessesary
                    true);                          // sets inverted search
                m_in_shadow[cam_index] = rsi.rp.has_value();

                completely_tested[x][y] = true;

                omp_set_lock(&lck);
                if (rsi.rp.has_value())
                {
                    ++num_found;
                    new_test = true;
                }
                cout << "\rIteration " << iteration << " | Shadows found: " << num_found << " / " << ++num_tested << flush;
                omp_unset_lock(&lck);
                TimerHandler::overall_timer().deleteTimer(tid);
            }
            ++iteration;
            num_found_total += num_found;
            num_tested = num_found = 0;
            cout << endl;
        }
        cout << "Total shadows found: " << num_found_total << endl;
        // delete used arrays
        for (size_t x = 0; x < m_cam_width; ++x)
            delete[] completely_tested[x];
        delete[] completely_tested;

        m_is_shadows_sharp = true;
        saveShadows();
    }

    // ------------------------------------------------------------------------- //
    bool Shader::loadNormals(NormalCalcStrategy strategy)
    {
        // catch invalid case
        if (strategy == NONE)
        {
            cout << "Did not load normals (type NONE is not valid for this action)" << endl;
            return false;
        }
        string save_location = getNormalSaveLocation(strategy);
        ifstream file{save_location};
        if (!file)
            return false;
        // prepare extraction into a temporary vector
        vector<Vec3r> temp_normals{m_normals};
        size_t count = 0, success_count = 0;
        Vec3r v{0, 0, 0};
        // write to v and set corresponding value in temp result
        while (file >> v && count < temp_normals.size())
        {
            temp_normals[count++] = v;
            if (v[0] != 0 || v[1] != 0 || v[2] != 0)
                ++success_count;
        }
        m_is_normals_ready = file.eof() && count == m_normals.size();
        // if yes: set new values
        if (m_is_normals_ready)
        {
            cout << "Loaded normals ("
                 << success_count << " / " 
                 << m_raytracer->getProgress().numPointsFound() 
                 << " were successfull)" << endl;
            m_current_normal_strategy = strategy;
            m_normals = temp_normals;
        }
        return m_is_normals_ready;
    }

    // ------------------------------------------------------------------------- //
    bool Shader::loadShadows()
    {
        string save_location = m_save_dir + "/in_shadow.txt";
        ifstream file{save_location};
        if (!file)
            return false;
        size_t count = 0;
        bool b;
        while (file >> b && count < m_normals.size())
            m_in_shadow[count++] = b;
        m_is_shadows_ready = file.eof() && count == m_in_shadow.size();
        return m_is_shadows_ready;
    }

    // ------------------------------------------------------------------------- //
    void Shader::saveNormals(NormalCalcStrategy strategy) const
    {
        if (!m_is_normals_ready || strategy == NONE)
            return;
        string save_location = getNormalSaveLocation(strategy);
        ofstream file{save_location};
        if (file)
            for (Vec3r v : m_normals)
                file << v[0] << ' ' << v[1] << ' ' << v[2] << "\n";
        file.close();
    }

    // ------------------------------------------------------------------------- //
    void Shader::saveShadows() const
    {
        if (!m_is_shadows_ready)
            return;
        string save_location = m_save_dir + "/in_shadow.txt";
        ofstream file{save_location};
        if (file)
            for (bool b : m_in_shadow)
                file << b << "\n";
        file.close();
    }

    // ------------------------------------------------------------------------- //
    string Shader::getNormalSaveLocation(NormalCalcStrategy strategy) const
    {
        if (strategy == NEIGHBORS)
            return m_save_dir + "/normals_ne.txt";
        if (strategy == SAMPLING)
            return m_save_dir + "/normals_sa.txt";
        if (strategy == HYBRID)
            return m_save_dir + "/normals_hy.txt";
        return "";
    }

    // ------------------------------------------------------------------------- //
    std::string Shader::getT0SaveLocation(NormalCalcStrategy strategy, bool shadow_on) const
    {
        // name dependent on exact configuration
        string save_location = m_save_dir + "/t0";

        if (strategy == NEIGHBORS)
            save_location += "_ne";
        else if (strategy == SAMPLING)
            save_location += "_sa";
        else if (strategy == HYBRID)
            save_location += "_hy";

        if (shadow_on)
        {
            if (m_is_shadows_sharp)
                return save_location + "_shad_sharp.ppm";
            return save_location + "_shad.ppm";
        }
        return save_location + ".ppm";
    }

    // ------------------------------------------------------------------------- //
    std::string Shader::getTauSaveLocation(NormalCalcStrategy strategy, bool shadow_on) const
    {
        // name dependent on exact configuration
        if (strategy == NONE)
            return "";
        string s = m_save_dir + "/tau_";
        if (strategy == NEIGHBORS)
            s += "ne";
        else if (strategy == SAMPLING)
            s += "sa";
        else
            s += "hy";
        if (shadow_on)
        {
            if (m_is_shadows_sharp)
                return s + "_shad_sharp.ppm";
            return s + "_shad.ppm";
        }
        return s + ".ppm";
    }

    // ------------------------------------------------------------------------- //
    bool Shader::isInShadow(const Vec3r &point) const
    {
        Ray light_ray{point, -m_light.light_direction_to(point)};

        // First: test all simple objects
        auto &objects = m_raytracer->getScene()->getObjects();
        for (auto &obj : objects)
            if (obj->getIntersection(light_ray, Globals::SMALL).has_value())
                return true;

        // Second: check the RecSurface
        RSIntersection rsi = m_raytracer->getScene()->getRecSurface().searchIntersection(
            light_ray,
            m_raytracer->getProgress(),
            *(m_raytracer->getCamera()),
            objects,
            Globals::RAYFOREOFFSET_SHADOWS);

        return rsi.rp.has_value();
    }

    // ------------------------------------------------------------------------- //
    Vec3r Shader::estimateNormalFromNeighbors(size_t cam_index) const
    {
        size_t x = cam_index % m_cam_width, y = cam_index / m_cam_width;
        if (y >= m_cam_height) // out of texture bounds
            return {0, 0, 0};
        const ProgressSaver &rt_progress = m_raytracer->getProgress();
        auto rsi = rt_progress.getRSI(x, y);
        if (!rsi)
            return {0, 0, 0};

        // get all neighboring RSIs ...
        const RSIntersection *neighbor_RSIs[4] = {
            rt_progress.getRSI(x, y - 1),
            rt_progress.getRSI(x - 1, y),
            rt_progress.getRSI(x, y + 1),
            rt_progress.getRSI(x + 1, y)};
        // ... and check if they are considered as 5D neighbors
        for (auto &n : neighbor_RSIs)
            if (n && !rsi->isNeighboring(*n))
                n = nullptr;

        Vec3r p0 = rsi->rp.value().pos;
        vector<Vec3r> n_normals{}; // calculated normals

        for (size_t i1 = 0; i1 < 4; ++i1)
        {
            size_t i2 = (i1 + 1) % 4;
            if (neighbor_RSIs[i1] && neighbor_RSIs[i2])
            {
                Vec3r p1 = neighbor_RSIs[i1]->rp.value().pos;
                Vec3r p2 = neighbor_RSIs[i2]->rp.value().pos;
                Vec3r n = surface_normal(p0, p1, p2);
                if ((n | rsi->ray.direction()) > 0) // flip normal if its orientation is false
                    n = -n;
                n_normals.push_back(n);
            }
        }
        // take average normal
        Vec3r normal = {0, 0, 0};
        for (size_t i = 0; i < n_normals.size(); ++i)
            normal += n_normals[i];
        // no normalization for zero vector!
        if (normal[0] == 0 && normal[1] == 0 && normal[2] == 0)
            return normal;
        return normal.normalize();
    }

    // ------------------------------------------------------------------------- //
    array<color, 2> Shader::shadeRecSurface(size_t cam_index, bool do_shading, bool do_shadows) const
    {
        size_t x = cam_index % m_cam_width, y = cam_index / m_cam_width;
        assert(y < m_cam_height);
        Vec2r uv{(double)x / (double)(m_cam_width - 1), (double)y / (double)(m_cam_height - 1)};
        const RSIntersection *rsi = m_raytracer->getProgress().getRSI(x, y);
        // special case 1: no intersection with RecSurface
        if (!rsi)
            return {m_back_col, m_back_col};
        // special case 2: no effects
        if (!do_shading && !do_shadows)
            return {m_phong_t0.sample(uv), m_phong_tau.sample(uv)};

        const Vec3r &n = m_normals[cam_index];
        // handle unsuccessfull normal calculation
        if (do_shading && n[0] == 0 && n[1] == 0 && n[2] == 0)
        {
            // for demonstration purposes red coloring if no shdows
            if (!do_shadows)
                return {red, red};
            // for "final" texture with shading and shadows: take normally sampled color
            else if (!m_in_shadow[cam_index])
                return {m_phong_t0.sample(uv), m_phong_tau.sample(uv)};
        }

        Intersection hit{
            nullptr,
            rsi->ray,
            rsi->hit.value(),
            rsi->rp->pos,
            n,
            uv};
        // shade shadow by setting normal to 0 (leads to ambient color only)
        if (do_shadows && m_in_shadow[cam_index])
            hit.normal = {0, 0, 0};
        // return shaded intersection
        return {m_phong_t0.shade(m_light, hit), m_phong_tau.shade(m_light, hit)};
    }

    // ------------------------------------------------------------------------- //
    color Shader::shadeCommonObjects(size_t cam_index, bool do_shading, bool do_shadows) const
    {
        size_t x = cam_index % m_cam_width, y = cam_index / m_cam_width;
        assert(y < m_cam_height);

        Ray ray = m_raytracer->getCamera()->ray(x, y);
        auto hit = m_raytracer->getScene()->getCommonObjectIntersection(ray);

        // if there is no value: just return background color
        if (!hit)
            return m_back_col;

        const Renderable *r = dynamic_cast<const Renderable *>(hit->intersectable);

        // if the point is in shadow: set normal to 0 which will render only ambient color
        bool in_shadow = do_shadows && m_in_shadow[cam_index];
        if (in_shadow)
            hit->normal = {0, 0, 0};
        // shade if the point is in shadow (leads to ambient coloring) or if it should be normally shaded
        if (in_shadow || do_shading)
            return r->shade(m_light, hit.value());
        // else return basic color
        return r->shade(m_light, hit.value());
    }
    // -------------------------------------------------------------------------- //
}
// ------------------------------------------------------------------------- //