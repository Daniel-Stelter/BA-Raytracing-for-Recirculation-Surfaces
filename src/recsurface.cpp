#include "recsurface.hh"

#include "line.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    void swapValue(Vec3r &v1, Vec3r &v2, int i)
    {
        real temp = v1[i];
        v1[i] = v2[i];
        v2[i] = temp;
    }

    //--------------------------------------------------------------------------//
    RecSurface::RecSurface(shared_ptr<Flow3D> p_flow,
                           const DataParams &data,
                           const SearchParams &search)
        : p_flow{p_flow}, m_data{data}, m_search{search} {}

    //--------------------------------------------------------------------------//
    optional<Vec2r> RecSurface::getDomainIntersections(const Ray &ray, real begin_at, real end_at) const
    {
        return m_data.domain.getIntersectionPositions(ray, begin_at);
    }

    //--------------------------------------------------------------------------//
    optional<RecPoint> RecSurface::getRecPoint(HyperLine &hl, const Ray &ray) const
    {
        vector<RecPoint> recPoints = hl.getRecirculationPoints(m_search, false);
        if (!recPoints.empty())
        {
            size_t min_id = 0;
            real min_dis = numeric_limits<real>::max();
            // search for the nearest point
            for (size_t i = 0; i < recPoints.size(); i++)
            {
                real dis = (recPoints[i].pos - ray.origin()).norm();
                if (dis < min_dis)
                {
                    min_id = i;
                    min_dis = dis;
                }
            }
            return recPoints[min_id];
        }
        return {};
    }

    //--------------------------------------------------------------------------//
    RSIntersection RecSurface::searchIntersection(const Ray &ray,
                                                  real begin_at,
                                                  real end_at,
                                                  bool *needed_integration) const
    {
        RSIntersection result{numeric_limits<size_t>::max(), ray, {}, {}};

        auto range = getDomainIntersections(ray, begin_at, end_at);
        // check if the domain is intersected
        if (!range)
        {
            if (needed_integration)
                *needed_integration = false;
        }
        else
        {
            if (needed_integration)
                *needed_integration = true;

            // setup some values and objects
            real &i_min = range.value()[0];
            real &i_max = range.value()[1];
            real step_size = m_data.step_size;

            FlowSampler3D sampler(*p_flow);
            Vec3r pA, pB = ray(i_min);
            HyperLine hl{pB, pB, &sampler}; // will be overwritten in first iteration

            // iterate over the ray
            for (real i = i_min; i < i_max; i += step_size)
            {
                pA = pB;
                pB = ray(std::min(i + step_size, i_max));
                hl = HyperLine(hl.getHyperPointB(), pB);

                if (p_flow->isInside(pA) && p_flow->isInside(pB))
                {
                    auto opt = getRecPoint(hl, ray);
                    if (opt.has_value())
                    {
                        result.rp = opt;
                        result.hit = (ray.origin() - opt->pos).norm();
                        break;
                    }
                }
            }
        }
        return result;
    }

    //--------------------------------------------------------------------------//
    RSIntersection RecSurface::searchIntersection(const Ray &ray,
                                                  const ProgressSaver &progress,
                                                  const Camera &cam,
                                                  const vector<unique_ptr<Renderable>> &objects,
                                                  real begin_at,
                                                  real end_at,
                                                  bool *needed_integration,
                                                  bool invert_search) const
    {
        RSIntersection result{numeric_limits<size_t>::max(), ray, {}, {}};

        auto range = getDomainIntersections(ray, begin_at, end_at);
        // check if the domain is intersected
        if (!range)
        {
            if (needed_integration)
                *needed_integration = false;
        }
        else
        {
            if (needed_integration)
                *needed_integration = true;

            real &i_min = range.value()[0];
            real &i_max = range.value()[1];
            real step_size = m_data.step_size;

            FlowSampler3D sampler(*p_flow);
            Vec3r pA, pB = ray(i_min);
            HyperLine hl{pB, pB, &sampler}; // will be overwritten in first iteration

            // iterate over ray
            for (real i = i_min; i < i_max; i += step_size)
            {
                pA = pB;
                pB = ray(std::min(i + step_size, i_max));
                hl = HyperLine(hl.getHyperPointB(), pB);

                if (p_flow->isInside(pA) && p_flow->isInside(pB))
                {
                    // determines if the line needs test depending on whether the search is inverted
                    if (doesLineNeedTest(pA, pB, cam, progress, objects) != invert_search)
                    {
                        auto opt = getRecPoint(hl, ray);
                        if (opt.has_value())
                        {
                            result.rp = opt;
                            result.hit = (ray.origin() - opt->pos).norm();
                            break;
                        }
                    }
                }
            }
        }
        return result;
    }

    //--------------------------------------------------------------------------//
    bool RecSurface::doesLineNeedTest(const Vec3r &pA,
                                      const Vec3r &pB,
                                      const Camera &cam,
                                      const ProgressSaver &progress,
                                      const vector<unique_ptr<Renderable>> &objects) const
    {
        Vec3r dir = pB - pA;
        real dir_len = dir.norm();
        // step 1: find calculated rays in neath
        //         therefore: find all pixel of the projection of the line
        Line2D line{cam.projection(pA), cam.projection(pB)};
        auto l_points = line.getLinePoints(1.2); // line thickness, consider also pixel in the neighborhood
        // step 2: find out if any ray had successfull search in front of the ray segment
        for (Vec2i p : l_points)
        {
            // step 2.1: search for cam ray intersections
            //           if none found: continue
            real min_t = numeric_limits<real>::max();
            const RSIntersection *orig_rsi = progress.getRSI(p[0], p[1]);
            if (orig_rsi)
                min_t = std::min(min_t, orig_rsi->hit.value());
            // even if no RP found: maybe the ray was only tested up to a scene object
            else
            {
                Ray cam_ray = cam.ray(p[0], p[1]);
                for (auto &obj : objects)
                {
                    auto opt = obj->getIntersection(cam_ray, Globals::SMALL);
                    if (opt.has_value() && opt->t < min_t)
                        min_t = opt->t;
                }
            }
            if (min_t == numeric_limits<real>::max())
                continue;

            // step 2.2: set reference distance for test to maximum distance of
            real dis = std::max((cam.ray(p[0], p[1]).origin() - pA).norm(),
                                (cam.ray(p[0], p[1]).origin() - pB).norm());

            // step 2.3: compare to rsi intersection
            //           dir_len is a safety distance
            if (min_t - dir_len <= dis)
                return true;
        }
        return false;
    }

    //--------------------------------------------------------------------------//
    Vec3r RecSurface::estimateFlowNormal(const RecPoint &rp,
                                         const Ray &ray,
                                         real offset_space,
                                         size_t max_steps_smaller) const
    {
        vector<Vec3r> points{};

        // begin with cross strategy
        addNeighborhoodByCross(rp, points, offset_space);
        // if needed, add points of cube strategy -> mixed form
        if (points.size() < 2)
            addNeighborhoodByCube(rp, points, offset_space);
        // if still too less points: decrease distance of lines
        if (points.size() < 2)
        {
            if (max_steps_smaller > 0)
                return estimateFlowNormal(rp, ray, offset_space / 2, max_steps_smaller - 1);
            else
                return Vec3r{0, 0, 0};
        }

        Vec3r normal{0, 0, 0};
        // create triangles with the center point and all pairs
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            const Vec3r &p1 = points[i];
            for (size_t j = i + 1; j < points.size(); ++j)
            {
                const Vec3r &p2 = points[j];
                Vec3r n = surface_normal(rp.pos, p1, p2);
                // check for an angle between -90 and 90 degree
                if ((n | ray.direction()) > 0)
                    n = -n;
                normal += n;
            }
        }
        // zero vector must not be normalized
        if (normal[0] == 0 && normal[1] == 0 && normal[2] == 0)
            return normal;
        // false orientation?
        if ((ray.direction() | normal) > 0)
            normal = -normal;
        return normal.normalize();
    }

    //--------------------------------------------------------------------------//
    void RecSurface::addHyperlineToList(HyperLine &hl,
                                        const RecPoint &rp,
                                        vector<Vec3r> &points,
                                        const SearchParams &sp) const
    {
        Vec3r pA = hl.getHyperPointA().getPos();
        Vec3r pB = hl.getHyperPointB().getPos();
        if (!p_flow->isInside(pA) || !p_flow->isInside(pB))
            return;
        auto found = hl.getRecirculationPoints(sp, false);
        // two conditions for each point to be added:
        // 1. t0 and tau values are near enough
        // 2. there is no point in 3d which is extremely near (check with SPACEEQUAL constant)
        for (auto &f : found)
        {
            real dis = (f.pos - rp.pos).norm();
            if (abs(f.t0 - rp.t0) > dis * Globals::NEIGHBOR_DIFT0_PERLU || abs(f.tau - rp.tau) > dis * Globals::NEIGHBOR_DIFTAU_PERLU) // check 1
                continue;
            bool do_insert = true;
            for (Vec3r &other : points)
            {
                if ((f.pos - other).norm() <= Globals::SPACEEQUAL) // check 2
                {
                    do_insert = false;
                    break;
                }
            }
            if (do_insert)
                points.push_back(f.pos);
        }
    }

    //--------------------------------------------------------------------------//
    void RecSurface::addNeighborhoodByCube(const RecPoint &rp,
                                           vector<Vec3r> &points,
                                           real offset_space) const
    {
        //    6----------7
        //   /|         /|
        //  / |        / |
        // 2--+-------3  |
        // |  4-------|--5
        // | /        | /
        // |/         |/
        // 0----------1

        // setup params by restricting the searched values
        real dif_t0 = 1.7320508075688772 * Globals::NEIGHBOR_DIFT0_PERLU; // the given val is aüürpx sqrt(3)
        real dif_tau = 1.7320508075688772 * Globals::NEIGHBOR_DIFTAU_PERLU;
        SearchParams sp{max(rp.t0 - dif_t0, m_search.t0_min),
                        min(rp.t0 + dif_t0, m_search.t0_max),
                        m_search.tau_min,
                        min(rp.tau + dif_tau, m_search.tau_max),
                        m_search.dt,
                        m_search.prec};

        FlowSampler3D sampler(*p_flow);
        // create all corners needed
        Vec3r corners[8];
        for (size_t i = 0; i < 8; ++i)
        {
            real x = i % 2 < 1 ? -1 : 1;
            real y = i % 4 < 2 ? -1 : 1;
            real z = i % 8 < 4 ? -1 : 1;
            corners[i] = Vec3r{x, y, z} * offset_space + rp.pos;
        }
        // the following part is not more encapsulated since in this way some calculations can be reused
        HyperLine edges_1[4] = {
            {corners[0], corners[1], &sampler},  // 0---1
            {corners[2], corners[3], &sampler},  // 2---3
            {corners[4], corners[5], &sampler},  // 4---5
            {corners[6], corners[7], &sampler}}; // 6---7
        for (size_t i = 0; i < 4; ++i)
            addHyperlineToList(edges_1[i], rp, points, sp);
        HyperLine edges_2[4] = {
            {edges_1[0].getHyperPointA(), edges_1[1].getHyperPointA()},  // 0---2
            {edges_1[0].getHyperPointB(), edges_1[1].getHyperPointB()},  // 1---3
            {edges_1[2].getHyperPointA(), edges_1[3].getHyperPointA()},  // 4---6
            {edges_1[2].getHyperPointB(), edges_1[3].getHyperPointB()}}; // 5---7
        for (size_t i = 0; i < 4; ++i)
            addHyperlineToList(edges_2[i], rp, points, sp);
        HyperLine edges_3[4] = {
            {edges_2[0].getHyperPointA(), edges_2[2].getHyperPointA()},  // 0---4
            {edges_2[1].getHyperPointA(), edges_2[3].getHyperPointA()},  // 1---5
            {edges_2[0].getHyperPointB(), edges_2[2].getHyperPointB()},  // 2---6
            {edges_2[1].getHyperPointB(), edges_2[3].getHyperPointB()}}; // 3---7
        for (size_t i = 0; i < 4; ++i)
            addHyperlineToList(edges_3[i], rp, points, sp);
    }

    //--------------------------------------------------------------------------//
    void RecSurface::addNeighborhoodByCross(const RecPoint &rp,
                                            vector<Vec3r> &points,
                                            real offset_space) const
    {
        //             2
        //            /|
        //   7-------/-------6
        //   | 9----/--+-----|-10
        //   |/    3   |     |/
        //   /     | X |     /    X is the center
        //  /|     |   1    /|
        // 8-+-----+-------X |
        //   4-----|-/-------11
        //         |/
        //         0

        // setup params by restricting the searched values
        real dif_t0 = M_SQRT2 * Globals::NEIGHBOR_DIFT0_PERLU;
        real dif_tau = M_SQRT2 * Globals::NEIGHBOR_DIFTAU_PERLU;
        SearchParams sp{max(rp.t0 - dif_t0, m_search.t0_min),
                        min(rp.t0 + dif_t0, m_search.t0_max),
                        m_search.tau_min,
                        min(rp.tau + dif_tau, m_search.tau_max),
                        m_search.dt,
                        m_search.prec};

        FlowSampler3D sampler(*p_flow);
        // setup needed corners
        Vec3r corners[12] = {
            Vec3r{0, -1, -1} * offset_space + rp.pos,
            Vec3r{0, 1, -1} * offset_space + rp.pos,
            Vec3r{0, 1, 1} * offset_space + rp.pos,
            Vec3r{0, -1, 1} * offset_space + rp.pos,
            Vec3r{-1, 0, -1} * offset_space + rp.pos,
            Vec3r{1, 0, -1} * offset_space + rp.pos,
            Vec3r{1, 0, 1} * offset_space + rp.pos,
            Vec3r{-1, 0, 1} * offset_space + rp.pos,
            Vec3r{-1, -1, 0} * offset_space + rp.pos,
            Vec3r{1, -1, 0} * offset_space + rp.pos,
            Vec3r{1, 1, 0} * offset_space + rp.pos,
            Vec3r{-1, 1, 0} * offset_space + rp.pos};

        for (size_t i = 0; i < 12; i += 4)
        {
            HyperLine hl{corners[i], corners[i + 1], &sampler};
            addHyperlineToList(hl, rp, points, sp);
            HyperPoint hp_start = hl.getHyperPointA();

            hl = HyperLine(hl.getHyperPointB(), corners[i + 2]);
            addHyperlineToList(hl, rp, points, sp);
            hl = HyperLine(hl.getHyperPointB(), corners[i + 3]);
            addHyperlineToList(hl, rp, points, sp);
            hl = HyperLine(hl.getHyperPointB(), hp_start);
            addHyperlineToList(hl, rp, points, sp);
        }
    }
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//