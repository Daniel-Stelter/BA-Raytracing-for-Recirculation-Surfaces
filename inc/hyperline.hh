#pragma once

#include <list>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "critextractor.hh"
#include "flowsampler.hh"
#include "globals.hh"
#include "hyperpoint.hh"
#include "jobparams.hh"
#include "recpoint.hh"
#include "types.hh"

#include "vectorcuboid.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class HyperLine
  {
  public:
    //--------------------------------------------------------------------------//
    struct RecursiveSearchParams
    {
      //--------------------------------------------------------------------------//
      RecursiveSearchParams(const real &t0_a,
                            const real &t0_b,
                            const real &tau_a,
                            const real &tau_b,
                            const Vec3r &point_a,
                            const Vec3r &point_b,
                            const real &prec)
          : t0_a(t0_a), t0_b(t0_b), tau_a(tau_a), tau_b(tau_b), point_a(point_a), point_b(point_b), prec(prec){};
      //--------------------------------------------------------------------------//
      real t0_a = 0.0;
      real t0_b = 0.0;
      real tau_a = 0.0;
      real tau_b = 0.0;
      Vec3r point_a = Vec3r(0.0, 0.0, 0.0);
      Vec3r point_b = Vec3r(0.0, 0.0, 0.0);
      real prec = Globals::SEARCHPREC;
      //--------------------------------------------------------------------------//
      bool operator==(const RecursiveSearchParams &other) const
      {
        return (t0_a == other.t0_a && t0_b == other.t0_b &&
                tau_a == other.tau_a && tau_b == other.tau_b &&
                prec == other.prec &&
                abs((point_a - other.point_a).norm()) < Globals::SMALL &&
                abs((point_b - other.point_b).norm()) < Globals::SMALL);
      }
      //--------------------------------------------------------------------------//
    };
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    HyperLine(const Vec3r &pointA,
              const Vec3r &pointB,
              FlowSampler3D *p_flowSampler);
    HyperLine(const Vec3r &point, const HyperPoint &hyperPoint);
    HyperLine(const HyperPoint &hyperPoint, const Vec3r &point);
    HyperLine(const HyperPoint &hyperPointA, const HyperPoint &hyperPointB);
    //--------------------------------------------------------------------------//
    virtual ~HyperLine(void);
    //--------------------------------------------------------------------------//
    std::vector<RecPoint> getRecirculationPoints(
        const real &t0_min,
        const real &t0_max,
        const real &tau_min,
        const real &tau_max,
        const real &dt,
        const real &prec,
        const bool &refine = false,
        std::vector<RecPoint> *p_candidates = nullptr,
        int *p_stopProcess = nullptr);
    //--------------------------------------------------------------------------//
    std::vector<RecPoint> getRecirculationPoints(
        const SearchParams &search,
        const bool &refine = false,
        std::vector<RecPoint> *p_candidates = nullptr,
        int *p_stopProcess = nullptr);
    //--------------------------------------------------------------------------//
    RecPoint createRecPoint(const Vec3r &pos,
                            const real &t0,
                            const real &tau) const;
    //--------------------------------------------------------------------------//
    HyperLine &operator=(const HyperLine &line);
    //--------------------------------------------------------------------------//
    const HyperPoint &getHyperPointA(void) const;
    const HyperPoint &getHyperPointB(void) const;
    FlowSampler3D *getFlowSampler(void) const;
    //--------------------------------------------------------------------------//
  private:
    //--------------------------------------------------------------------------//
    bool getDiffVector(Vec3r *diffVec,
                       const std::shared_ptr<FlowMap3D> *p_flowMaps,
                       const Vec3r &point_a,
                       const Vec3r &point_b,
                       const real &tau_a,
                       const real &tau_b) const;
    //--------------------------------------------------------------------------//
    // subsampling search
    typedef std::pair<RecursiveSearchParams,
                      std::vector<std::shared_ptr<FlowMap3D>>>
        SearchPair;
    //--------------------------------------------------------------------------//
    /** \param p_stopProcess - Flag, which is 1, when the process takes to long and
     * should be killed, and 0 else. This is not a bool, due to inability to
     * a vector<bool>.*/
    std::list<RecPoint> searchRecPointSampling(
        const RecursiveSearchParams &params,
        const std::shared_ptr<FlowMap3D> *p_flowMaps,
        real reqDist = -1.0,
        int level = 0,
        int *p_stopProcess = nullptr);
    //--------------------------------------------------------------------------//
    std::list<RecPoint> searchRecPointSamplingIter(
        const RecursiveSearchParams &params,
        const std::shared_ptr<FlowMap3D> *p_flowMaps,
        real reqDist = -1.0,
        int *p_stopProcess = nullptr);
    //--------------------------------------------------------------------------//
    void resampleCuboid(const SearchPair &searchSpace,
                        std::vector<std::shared_ptr<FlowMap3D>> &subFlowMaps,
                        std::list<SearchPair> *p_searchList);
    //--------------------------------------------------------------------------//
    bool reachedSamplingAccuracy(const RecursiveSearchParams &params,
                                 bool *p_time = nullptr,
                                 bool *p_space = nullptr) const;
    //--------------------------------------------------------------------------//
    void refineSearchSpace(const RecursiveSearchParams &params,
                           const bool &refineSpace,
                           const bool &refineTime,
                           const std::shared_ptr<FlowMap3D> *p_flowMaps,
                           std::shared_ptr<FlowMap3D> *p_subFlowMaps,
                           std::vector<RecursiveSearchParams> *p_subParams,
                           std::vector<std::vector<int>> *p_mapIndexes);
    //--------------------------------------------------------------------------//
    std::list<std::pair<int, bool>> computePreferedOctants(
        std::shared_ptr<FlowMap3D> *p_subFlowMaps,
        std::vector<RecursiveSearchParams> *p_subParams,
        std::vector<std::vector<int>> *p_mapIndexes) const;
    //--------------------------------------------------------------------------//
    Vec3r m_pointA;
    Vec3r m_pointB;
    bool m_refine = false; // true, when search process is used for refinment

    HyperPoint m_hyperPointA;
    HyperPoint m_hyperPointB;
    FlowSampler3D *mp_flowSampler;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//