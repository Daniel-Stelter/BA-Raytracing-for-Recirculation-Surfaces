#include "scene.hh"

#include "colormap.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
    // ------------------------------------------------------------------------- //
    Vec2i discretize(const Vec2i &v) { return {(int)round(v[0]), (int)round(v[1])}; }

    // ------------------------------------------------------------------------- //
    Scene::Scene(const RecSurface &rec_surface,
                 const Vec3r &light_dir,
                 const color &background)
        : m_rec_surface{rec_surface},
          m_light_direction{light_dir},
          m_background{background} {}

    //--------------------------------------------------------------------------//
    color Scene::t0Color(const real &t0) const
    {
        real percentage = t0 / m_rec_surface.getSearchParams().t0_max;
        return getColorViridis(percentage);
    }

    //--------------------------------------------------------------------------//
    color Scene::tauColor(const real &tau) const
    {
        real percentage = tau / m_rec_surface.getSearchParams().tau_max;
        return getColorInferno(percentage);
    }

    //--------------------------------------------------------------------------//
    optional<Intersection> Scene::getCommonObjectIntersection(const Ray &ray,
                                                              real begin_at) const
    {
        optional<Intersection> result{};
        // observe nearest intersection
        real min_t = numeric_limits<real>::max();
        for (auto &obj : m_objects)
        {
            auto opt = obj->getIntersection(ray, begin_at);
            if (opt && opt->t < min_t)
            {
                min_t = opt->t;
                result = opt;
            }
        }
        return result;
    }

    //--------------------------------------------------------------------------//
    color Scene::raytracingCommonObjects(const Ray &ray, real begin_at)
    {
        auto obj_hit = getCommonObjectIntersection(ray, begin_at);
        if (obj_hit)
            return dynamic_cast<const Renderable *>(obj_hit->intersectable)->sample(obj_hit->uv);
        return m_background;
    }

    //--------------------------------------------------------------------------//
    array<color, 2> Scene::raytracing(const Ray &ray,
                                      RSIntersection &rsi_result,
                                      real begin_at,
                                      real end_at)
    {
        bool unused;
        return raytracing(ray, rsi_result, unused, begin_at, end_at);
    }

    //--------------------------------------------------------------------------//
    array<color, 2> Scene::raytracing(const Ray &ray,
                                      RSIntersection &rsi_result,
                                      bool &rs_domain_intersected,
                                      real begin_at,
                                      real end_at)
    {
        array<color, 2> colors = {m_background, m_background};

        // find first hit with a common object
        real min_t = numeric_limits<real>::max();
        auto obj_hit = getCommonObjectIntersection(ray, begin_at);
        if (obj_hit)
        {
            min_t = obj_hit->t;
            color c = dynamic_cast<const Renderable *>(obj_hit->intersectable)->sample(obj_hit->uv);
            colors = {c, c};
        }
        // might not be needed to test the full ray
        end_at = min(end_at, min_t);

        RSIntersection rsi = m_rec_surface.searchIntersection(ray, begin_at, end_at, &rs_domain_intersected);
        if (rsi.rp.has_value())
        {
            rsi_result.rp = rsi.rp;
            rsi_result.hit = rsi.hit;
            colors = {t0Color(rsi.rp->t0), tauColor(rsi.rp->tau)};
        }

        return colors;
    }
    // ------------------------------------------------------------------------- //
}
// ------------------------------------------------------------------------- //