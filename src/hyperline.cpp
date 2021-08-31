#include "hyperline.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  const HyperPoint &
  HyperLine::getHyperPointA(void) const
  {
    return m_hyperPointA;
  }

  //--------------------------------------------------------------------------//
  const HyperPoint &
  HyperLine::getHyperPointB(void) const
  {
    return m_hyperPointB;
  }

  //--------------------------------------------------------------------------//
  FlowSampler3D *
  HyperLine::getFlowSampler(void) const
  {
    return mp_flowSampler;
  }

  //--------------------------------------------------------------------------//
  HyperLine::HyperLine(const Vec3r &pointA,
                       const Vec3r &pointB,
                       FlowSampler3D *p_flowSampler)
      : m_pointA(pointA), m_pointB(pointB), m_hyperPointA(m_pointA, p_flowSampler), m_hyperPointB(m_pointB, p_flowSampler), mp_flowSampler(p_flowSampler) {}

  //--------------------------------------------------------------------------//
  HyperLine::HyperLine(const Vec3r &point, const HyperPoint &hyperPoint)
      : m_pointA(point), m_pointB(hyperPoint.getPos()), m_hyperPointA(m_pointA, hyperPoint.getFlowSampler()), m_hyperPointB(hyperPoint), mp_flowSampler(hyperPoint.getFlowSampler()) {}

  //--------------------------------------------------------------------------//
  HyperLine::HyperLine(const HyperPoint &hyperPoint, const Vec3r &point)
      : m_pointA(hyperPoint.getPos()), m_pointB(point), m_hyperPointA(hyperPoint), m_hyperPointB(m_pointB, hyperPoint.getFlowSampler()), mp_flowSampler(hyperPoint.getFlowSampler()) {}

  //--------------------------------------------------------------------------//
  HyperLine::HyperLine(const HyperPoint &hyperPointA,
                       const HyperPoint &hyperPointB)
      : m_pointA(hyperPointA.getPos()), m_pointB(hyperPointB.getPos()), m_hyperPointA(hyperPointA), m_hyperPointB(hyperPointB), mp_flowSampler(hyperPointA.getFlowSampler()) {}

  //--------------------------------------------------------------------------//
  HyperLine &
  HyperLine::operator=(const HyperLine &line)
  {
    this->mp_flowSampler = line.mp_flowSampler;
    this->m_pointA = line.m_pointA;
    this->m_pointB = line.m_pointB;
    this->m_hyperPointA = line.m_hyperPointA;
    this->m_hyperPointB = line.m_hyperPointB;
    return *this;
  }

  //--------------------------------------------------------------------------//
  HyperLine::~HyperLine(void) {}

  //--------------------------------------------------------------------------//
  std::vector<RecPoint>
  HyperLine::getRecirculationPoints(const real &t0_min,
                                    const real &t0_max,
                                    const real &tau_min,
                                    const real &tau_max,
                                    const real &dt,
                                    const real &prec,
                                    const bool &refine,
                                    std::vector<RecPoint> *p_candidates,
                                    int *p_stopProcess)
  {
    m_refine = refine;

    // output container for the recirculation points
    std::list<RecPoint> recPoints;

    // some infos and containers needed for the computation
    real t0_a = t0_min;
    real t0_b = std::min(t0_min + dt, t0_max);
    Vec3r diffVec[8];

    shared_ptr<FlowMap3D> flowMaps[4];

    // prepare an array with the needed flowmaps for the first loop
    flowMaps[0] = m_hyperPointA.getFlowMap(t0_a, tau_max);
    flowMaps[1] = m_hyperPointB.getFlowMap(t0_a, tau_max);
    flowMaps[2] = flowMaps[0];
    flowMaps[3] = flowMaps[1];

    // loop over the possible t0
    while (t0_a < t0_max)
    {
      // in each loop we use two old maps and need two new ones
      flowMaps[0] = flowMaps[2];
      flowMaps[1] = flowMaps[3];
      flowMaps[2] = m_hyperPointA.getFlowMap(t0_b, tau_max);
      flowMaps[3] = m_hyperPointB.getFlowMap(t0_b, tau_max);

      real tau_a = tau_min; // tau must not be 0.0
      real tau_b = std::min(tau_min + dt, tau_max);

      // loop over tau
      while (tau_a < tau_max)
      {
        // tau_a and tau_b must not be 0.0
        if (abs(tau_a) < Globals::TAUMIN)
          tau_a = std::copysign(Globals::TAUMIN, tau_a);
        if (abs(tau_b) < Globals::TAUMIN)
          tau_b = std::copysign(Globals::TAUMIN, tau_b);
        if (Globals::ZERO > abs(tau_b - tau_a))
          break;

        // Check one of the points is OutOfDomain.
        if (false ==
            getDiffVector(
                &diffVec[0], &flowMaps[0], m_pointA, m_pointB, tau_a, tau_b))
        {
          tau_a = tau_b;
          tau_b = std::min(tau_b + dt, tau_max);
          continue;
        }
        // compute the scaling for this cube
        real scale[3];
        scale[0] = abs(tau_b - tau_a);
        scale[1] = (m_pointA - m_pointB).norm2();
        scale[2] = abs(t0_b - t0_a);
        // check if the eight corresponding points have possible RecPoint in them
        VectorCuboid checkCube(&diffVec[0], &scale[0]);
        RecursiveSearchParams param(
            t0_a, t0_b, tau_a, tau_b, m_pointA, m_pointB, prec);
        if (checkCube.passesSignTest())
        {
          // start the recursive search to find the RecPoints
          auto tempRecPoints = std::list<RecPoint>();
          /**We have two possibilties recursive or iterative search.**/
          /* 1. recursive search*/
          //  tempRecPoints = searchRecPointSampling(param, &flowMaps[0],
          //     Globals::SPACEEQUAL, 0, p_stopProcess);

          /* 2. iterative search */
          tempRecPoints = searchRecPointSamplingIter(
              param, flowMaps, Globals::SPACEEQUAL, p_stopProcess);

          // insert the found RecPoints for the current t0/tau combination
          if (0 < tempRecPoints.size())
          {
            if (nullptr != p_candidates)
              (*p_candidates)
                  .insert(p_candidates->end(),
                          tempRecPoints.begin(),
                          tempRecPoints.end());
            // assumption: a line hits only one point
            // -> for a (t0, tau)-combination, we pick the point,
            // with the best recirculation properties
            RecPoint &bestPnt = tempRecPoints.front();
            for (auto &tempPnt : tempRecPoints)
              if (tempPnt.dist < bestPnt.dist)
                bestPnt = tempPnt;
            recPoints.push_back(bestPnt);
          }
        }
        // if the process got killed for this line, return the current found
        // points
        if (p_stopProcess && 0 != *p_stopProcess)
        {
          std::vector<RecPoint> recPointsVec;
          recPointsVec.insert(
              recPointsVec.begin(), recPoints.begin(), recPoints.end());
          return recPointsVec;
        }
        tau_a = tau_b;
        tau_b = std::min(tau_b + dt, tau_max);
      }
      t0_a = t0_b;
      t0_b = std::min(t0_b + dt, t0_max);
    }
    // convert the fast map to a vector
    std::vector<RecPoint> recPointsVec;
    recPointsVec.insert(recPointsVec.begin(), recPoints.begin(), recPoints.end());

    return recPointsVec;
  }
  std::vector<RecPoint>
  HyperLine::getRecirculationPoints(const SearchParams &search,
                                    const bool &refine,
                                    std::vector<RecPoint> *p_candidates,
                                    int *p_stopProcess)
  {
    return getRecirculationPoints(
        search.t0_min,
        search.t0_max,
        search.tau_min,
        search.tau_max,
        search.dt,
        search.prec,
        refine,
        p_candidates,
        p_stopProcess);
  }

  //--------------------------------------------------------------------------//
  /**Does a recursive subdivision of the search space, that is defined by \c param
  to find critical points / recirculation points. When this method is called and
  the search parameters define a search space, that is smaller, than the wanted
  precision, the center of this search area is returned as a recirculation
  point.
  \param level - the current recursion depth
  \param reqDist - \li if < 0.0, all RecPoints are searched
                 - \li if > 0.0 the search is stopped as soon as a RecPoint with
                       dist < reqDist is found */
  std::list<RecPoint>
  HyperLine::searchRecPointSampling(const RecursiveSearchParams &params,
                                    const std::shared_ptr<FlowMap3D> *p_flowMaps,
                                    real reqDist,
                                    int level,
                                    int *p_stopProcess)
  {
    auto recPoints = std::list<RecPoint>();
    real t0_a = params.t0_a;
    real t0_b = params.t0_b;
    real t0_avg = (t0_a + t0_b) / 2.0;
    real tau_a = params.tau_a;
    real tau_b = params.tau_b;
    real tau_avg = (tau_a + tau_b) / 2.0;
    Vec3r point_a = params.point_a;
    Vec3r point_b = params.point_b;
    Vec3r point_avg = (point_a + point_b) / 2.0;

    // check if we reached the wanted accuracy
    bool space = false;
    bool time = false;
    if (reachedSamplingAccuracy(params, &time, &space))
    {
      // if so average the point from the available infos and return it
      recPoints.push_back(createRecPoint(point_avg, t0_avg, tau_avg));
      return recPoints;
    }

    // kill the process if it takes too long
    if (p_stopProcess && *p_stopProcess)
      return recPoints;

    // do a further resampling of the searchspace
    std::shared_ptr<FlowMap3D> flowMaps[9];

    auto subPara = std::vector<RecursiveSearchParams>();
    auto mapIndi = std::vector<std::vector<int>>();
    refineSearchSpace(
        params, !space, !time, p_flowMaps, &flowMaps[0], &subPara, &mapIndi);

    // estimate a good search order for the octants
    std::list<std::pair<int, bool>> octants =
        computePreferedOctants(&flowMaps[0], &subPara, &mapIndi);

    // search the subspaces with the corresponding parameters and flowmaps
    std::list<RecPoint> tempRecPoints;
    std::shared_ptr<FlowMap3D> subMaps[4];
    for (auto &octant : octants)
    {
      subMaps[0] = flowMaps[mapIndi[octant.first][0]];
      subMaps[1] = flowMaps[mapIndi[octant.first][1]];
      subMaps[2] = flowMaps[mapIndi[octant.first][2]];
      subMaps[3] = flowMaps[mapIndi[octant.first][3]];
      tempRecPoints = searchRecPointSampling(
          subPara[octant.first], &subMaps[0], reqDist, (level + 1), p_stopProcess);
      if (reqDist < 0)
        recPoints.splice(recPoints.end(), tempRecPoints);
      else
        // transfer the found points, to the list with all points
        while (!tempRecPoints.empty())
        {
          auto rcp = tempRecPoints.front();
          tempRecPoints.pop_front();
          recPoints.push_front(rcp);
          // until we found a 'good enough' point which is pushed
          // to the front, to speed up merge on parent level
          if (rcp.dist < reqDist)
          {
            recPoints.splice(recPoints.end(), tempRecPoints);
            return recPoints;
          }
        }
    }
    return recPoints;
  }

  //--------------------------------------------------------------------------//
  /**This is the iterative version of the search method. The areas in which the
   (prefered) search takes place are managed by lists.  */
  std::list<RecPoint>
  HyperLine::searchRecPointSamplingIter(
      const RecursiveSearchParams &params,
      const std::shared_ptr<FlowMap3D> *p_flowMaps,
      real reqDist,
      int *p_stopProcess)
  {
    list<RecPoint> recPoints;
    // create a list with all octants that should be searched
    list<SearchPair> searchList;

    // put in the first entry, that gets searched
    vector<shared_ptr<FlowMap3D>> flowMaps4;
    for (int i = 0; i < 4; i++)
      flowMaps4.push_back(p_flowMaps[i]);
    auto entry = std::make_pair(params, flowMaps4);
    searchList.push_back(entry);

    // criteria to stop the search process
    unsigned int stepCount = 0;
    // unsigned int maxSteps = static_cast<int>(pow(8, 8));
    // unsigned int maxSteps = static_cast<int>(pow(8, 4));
    unsigned int maxSteps = 512;
    bool goOn = true;

    vector<shared_ptr<FlowMap3D>> flowMaps;

    // remind the old front element, to check if the trilinear search found a
    // RecPoint if so, the front element will change
    auto oldFront = searchList.front().first;

    // do a search over all elements
    while (goOn)
    {
      // get the next element
      auto entry =
          std::make_pair(searchList.front().first, searchList.front().second);
      searchList.pop_front();

      // check if we reached the wanted accuracy
      if (reachedSamplingAccuracy(entry.first))
      {
        // if so average the point from the available infos and return it
        real t0_avg = (entry.first.t0_a + entry.first.t0_b) / 2.0;
        real tau_avg = (entry.first.tau_a + entry.first.tau_b) / 2.0;
        Vec3r point_avg = (entry.first.point_a + entry.first.point_b) / 2.0;
        auto rcp = createRecPoint(point_avg, t0_avg, tau_avg);
        // cout << "found point. step count: " << stepCount << "\n";
        if (reqDist < 0 || reqDist < rcp.dist)
          recPoints.push_back(rcp);
        else
        {
          recPoints.push_front(rcp);
          // cout << "found point. Steps needed:     " << stepCount << "\n";
          return recPoints;
        }
      }

      // remind the front element to check if trilinear search found a RecPoint
      if (!searchList.empty())
        oldFront = searchList.front().first;

      // if not resample the search space for this cube
      vector<shared_ptr<FlowMap3D>> subFlowMaps;
      subFlowMaps.resize(9);
      resampleCuboid(entry, subFlowMaps, &searchList);

      // check if the front element has changed, if so a prefered octant was
      // found if not we searched an area, where it is likely that there is no
      // point
      if (!searchList.empty() && oldFront == searchList.front().first)
      {
        stepCount++;
      }

      // copy references to keep flowmaps alive
      for (auto fm : subFlowMaps)
        if (fm)
          flowMaps.push_back(fm);

      // and go on
      goOn = !searchList.empty() && !(p_stopProcess && (0 != *p_stopProcess)) &&
             (stepCount < maxSteps);
    }
    // if (stepCount >= maxSteps)
    //   cout << "not found: " << stepCount << " stopped with maximum steps.\n";
    if (p_stopProcess && (0 != *p_stopProcess))
      cout << "not found: " << stepCount << " stopped by kill. " << *p_stopProcess
           << "\n";
    return recPoints;
  }

  //--------------------------------------------------------------------------//
  void
  HyperLine::resampleCuboid(const SearchPair &searchSpace,
                            vector<shared_ptr<FlowMap3D>> &subFlowMaps,
                            list<SearchPair> *p_searchList)
  {
    const RecursiveSearchParams &params = searchSpace.first;
    const std::shared_ptr<FlowMap3D> p_flowMaps = searchSpace.second[0];

    // real t0_a = params.t0_a;
    // real t0_b = params.t0_b;
    // real t0_avg = (t0_a + t0_b) / 2.0;
    // real tau_a = params.tau_a;
    // real tau_b = params.tau_b;
    // real tau_avg = (tau_a + tau_b) / 2.0;
    Vec3r point_a = params.point_a;
    Vec3r point_b = params.point_b;
    Vec3r point_avg = (point_a + point_b) / 2.0;

    // check if we reached the wanted accuracy
    bool space = false;
    bool time = false;
    if (reachedSamplingAccuracy(params, &time, &space))
      return;

    // do a further resampling of the searchspace
    auto subPara = std::vector<RecursiveSearchParams>();
    auto mapIndi = std::vector<std::vector<int>>();

    refineSearchSpace(params,
                      !space,
                      !time,
                      &searchSpace.second[0],
                      &subFlowMaps[0],
                      &subPara,
                      &mapIndi);

    // estimate a good search order for the octants
    std::list<std::pair<int, bool>> octants =
        computePreferedOctants(&subFlowMaps[0], &subPara, &mapIndi);

    // place the octants into the search list according to their preference
    vector<shared_ptr<FlowMap3D>> subMaps;
    subMaps.resize(4);
    for (auto &octant : octants)
    {
      subMaps[0] = subFlowMaps[mapIndi[octant.first][0]];
      subMaps[1] = subFlowMaps[mapIndi[octant.first][1]];
      subMaps[2] = subFlowMaps[mapIndi[octant.first][2]];
      subMaps[3] = subFlowMaps[mapIndi[octant.first][3]];

      if (octant.second)
        p_searchList->push_front(std::make_pair(subPara[octant.first], subMaps));
      else if (!m_refine)
        p_searchList->push_back(std::make_pair(subPara[octant.first], subMaps));
    }
  }

  //--------------------------------------------------------------------------//
  /**Generate the parameters for recursive subdivision.
\param [in] params - search parameters (t0, tau, position) for the current area
that get divided \param [in] refineSpace - is true, if the space component
should be subdivided \param [in] refineTime - is true, if the time component
should be subdivided \param [out] p_subFlowMaps - up to nine flowmaps, which
resample the divided space \param [out] p_subParams - the search parameters for
the each of the subspaces \param [out] p_mapIndexes - the flowmap indexes, that
belong to the parameters of the subspaces
*/
  void
  HyperLine::refineSearchSpace(const RecursiveSearchParams &params,
                               const bool &refineSpace,
                               const bool &refineTime,
                               const std::shared_ptr<FlowMap3D> *p_flowMaps,
                               std::shared_ptr<FlowMap3D> *p_subFlowMaps,
                               std::vector<RecursiveSearchParams> *p_subParams,
                               std::vector<std::vector<int>> *p_mapIndexes)
  {
    auto recPoints = std::list<RecPoint>();
    real t0_a = params.t0_a;
    real t0_b = params.t0_b;
    real t0_avg = (t0_a + t0_b) / 2.0;
    real tau_a = params.tau_a;
    real tau_b = params.tau_b;
    real tau_avg = (tau_a + tau_b) / 2.0;
    Vec3r point_a = params.point_a;
    Vec3r point_b = params.point_b;
    real prec = params.prec;
    Vec3r point_avg = (point_a + point_b) / 2.0;
    std::vector<RecursiveSearchParams> &subPara = *p_subParams;
    std::vector<std::vector<int>> &mapIndi = *p_mapIndexes;

    //----------//
    // refine space AND time
    if (refineSpace && refineTime)
    {
      // there are 9 new flowMaps that need to be computed
      for (int i = 0; i < 4; i++)
        p_subFlowMaps[i] = p_flowMaps[i];
      for (int i = 4; i < 9; i++)
        p_subFlowMaps[i] = make_shared<FlowMap3D>();
      mp_flowSampler->sampleFlow(p_subFlowMaps[4].get(), point_avg, t0_a, tau_b);
      mp_flowSampler->sampleFlow(p_subFlowMaps[5].get(), point_a, t0_avg, tau_b);
      mp_flowSampler->sampleFlow(
          p_subFlowMaps[6].get(), point_avg, t0_avg, tau_b);
      mp_flowSampler->sampleFlow(p_subFlowMaps[7].get(), point_b, t0_avg, tau_b);
      mp_flowSampler->sampleFlow(p_subFlowMaps[8].get(), point_avg, t0_b, tau_b);

      // prepare two vectors with time info and FlowMap indices for the next
      // subdivision step
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_avg, tau_a, tau_avg, point_a, point_avg, prec));
      mapIndi.push_back({0, 4, 5, 6});
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_avg, tau_avg, tau_b, point_a, point_avg, prec));
      mapIndi.push_back({0, 4, 5, 6});
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_avg, tau_a, tau_avg, point_avg, point_b, prec));
      mapIndi.push_back({4, 1, 6, 7});
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_avg, tau_avg, tau_b, point_avg, point_b, prec));
      mapIndi.push_back({4, 1, 6, 7});
      subPara.push_back(RecursiveSearchParams(
          t0_avg, t0_b, tau_a, tau_avg, point_a, point_avg, prec));
      mapIndi.push_back({5, 6, 2, 8});
      subPara.push_back(RecursiveSearchParams(
          t0_avg, t0_b, tau_avg, tau_b, point_a, point_avg, prec));
      mapIndi.push_back({5, 6, 2, 8});
      subPara.push_back(RecursiveSearchParams(
          t0_avg, t0_b, tau_a, tau_avg, point_avg, point_b, prec));
      mapIndi.push_back({6, 7, 8, 3});
      subPara.push_back(RecursiveSearchParams(
          t0_avg, t0_b, tau_avg, tau_b, point_avg, point_b, prec));
      mapIndi.push_back({6, 7, 8, 3});
    }
    //----------//
    // refine time only
    if (!refineSpace && refineTime)
    {
      // there are 9 new flowMaps that need to be computed
      for (int i = 0; i < 4; i++)
        p_subFlowMaps[i] = p_flowMaps[i];
      for (int i = 4; i < 9; i++)
        p_subFlowMaps[i] = make_shared<FlowMap3D>();
      mp_flowSampler->sampleFlow(p_subFlowMaps[5].get(), point_a, t0_avg, tau_b);
      mp_flowSampler->sampleFlow(p_subFlowMaps[7].get(), point_b, t0_avg, tau_b);

      // prepare two vectors with time info and FlowMap indices for the next
      // subdivision step
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_avg, tau_a, tau_avg, point_a, point_b, prec));
      mapIndi.push_back({0, 1, 5, 7});
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_avg, tau_avg, tau_b, point_a, point_b, prec));
      mapIndi.push_back({0, 1, 5, 7});
      subPara.push_back(RecursiveSearchParams(
          t0_avg, t0_b, tau_a, tau_avg, point_a, point_b, prec));
      mapIndi.push_back({5, 7, 2, 3});
      subPara.push_back(RecursiveSearchParams(
          t0_avg, t0_b, tau_avg, tau_b, point_a, point_b, prec));
      mapIndi.push_back({5, 7, 2, 3});
    }
    //----------//
    // refine space only
    if (refineSpace && !refineTime)
    {
      // there are 9 new flowMaps that need to be computed
      for (int i = 0; i < 4; i++)
        p_subFlowMaps[i] = p_flowMaps[i];
      for (int i = 4; i < 9; i++)
        p_subFlowMaps[i] = make_shared<FlowMap3D>();
      mp_flowSampler->sampleFlow(p_subFlowMaps[4].get(), point_avg, t0_a, tau_b);
      mp_flowSampler->sampleFlow(p_subFlowMaps[8].get(), point_avg, t0_b, tau_b);

      // prepare two vectors with time info and FlowMap indices for the next
      // subdivision step
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_b, tau_a, tau_b, point_a, point_avg, prec));
      mapIndi.push_back({0, 4, 2, 8});
      subPara.push_back(RecursiveSearchParams(
          t0_a, t0_b, tau_a, tau_b, point_a, point_avg, prec));
      mapIndi.push_back({4, 1, 8, 3});
    }
  }

  //--------------------------------------------------------------------------//
  /***Gets a octant which should be searched for RecPoints. The check is done by a
trilinear search. If this search reveals, there is a point, this octant will be
prefered for the search. If there is no point in there, this octant will not be
prefered. If there is an extended critical structure, this octant will be
skipped.
\return A list, which contains the octants, that will be searched and a
        flag indicating if it should be prefered (true) or not (false).*/
  std::list<std::pair<int, bool>>
  HyperLine::computePreferedOctants(
      std::shared_ptr<FlowMap3D> *p_subFlowMaps,
      std::vector<RecursiveSearchParams> *p_subParams,
      std::vector<std::vector<int>> *p_mapIndexes) const
  {
    std::list<std::pair<int, bool>> octants;
    int i = -1;
    std::shared_ptr<FlowMap3D> subMaps[4];
    for (auto sp : *p_subParams)
    {
      ++i;
      // extract the flowmaps of the subspace
      subMaps[0] = p_subFlowMaps[(*p_mapIndexes)[i][0]];
      subMaps[1] = p_subFlowMaps[(*p_mapIndexes)[i][1]];
      subMaps[2] = p_subFlowMaps[(*p_mapIndexes)[i][2]];
      subMaps[3] = p_subFlowMaps[(*p_mapIndexes)[i][3]];
      // check if the investigated sub space contains a critical point
      Vec3r diffVec[8];
      if (false ==
          getDiffVector(
              &diffVec[0], &subMaps[0], sp.point_a, sp.point_b, sp.tau_a, sp.tau_b))
        continue;
      real scale[3];
      scale[0] = abs(sp.tau_b - sp.tau_a);
      scale[1] = abs(subMaps[2]->t.front() - subMaps[0]->t.front());
      scale[2] = (sp.point_a - sp.point_b).norm2();
      VectorCuboid checkCube(&diffVec[0], &scale[0]);

      // passing the sign test is a must
      if (!checkCube.passesSignTest())
        continue;

      //--- check the cube for critical elements
      CritElements crEl;
      // we test for critical point via trilinear search - it can happen, that
      // points get missed due to trilinear interpolation. In this case
      // sign-Test is relevant only.
      if (!CritExtractor::hasCritElements(checkCube, &crEl))
      {
        octants.push_back(make_pair(i, false));
        continue;
      }

      // critical points leads to prefered computation
      if (crEl.containsCritPoints())
      {
        octants.push_front(make_pair(i, true));
        continue;
      }

      // extended critical structure leads to skipping this cube (just for
      // completness)
      if (crEl.containsCritStructures())
        continue;
    }
    return octants;
  }

  //--------------------------------------------------------------------------//
  bool
  HyperLine::reachedSamplingAccuracy(const RecursiveSearchParams &params,
                                     bool *p_time,
                                     bool *p_space) const
  {
    auto t0_dim = (params.t0_b - params.t0_a);
    auto tau_dim = (params.tau_b - params.tau_a);
    auto dist = (params.point_b - params.point_a).norm();

    bool tempSpace = false;
    bool tempTime = false;
    bool &space = p_space ? *p_space : tempSpace;
    bool &time = p_time ? *p_time : tempTime;

    // check if we reached the max subdiv level
    if (params.prec > t0_dim && params.prec > tau_dim)
      time = true;
    if (params.prec > dist)
      space = true;

    return (time && space);
  }

  //--------------------------------------------------------------------------//
  /**
Condition 1: out of domain
Condition 2: A particle, must reach different positions for different t0/tau
             combinations. If not it stays at its position.
\param p_diffVec pointer to the first element of an Vec3r[8]
\return false condition 1 or 2 is met.*/
  bool
  HyperLine::getDiffVector(Vec3r *diffVec,
                           const std::shared_ptr<FlowMap3D> *p_flowMaps,
                           const Vec3r &point_a,
                           const Vec3r &point_b,
                           const real &tau_a,
                           const real &tau_b) const
  {
    /*!**************************!*/
    /*! p_flowMaps[0]->t.front() !*/
    /*! equals the start time t0 !*/
    /*!**************************!*/

    // check if one of the points is out of domain
    // if the particle hits the boundary or a critcal point, it is not integrated
    // further in this case, the last time step is smaller, than the one which is
    // asked for or the flowmapis empty at all
    for (int i = 0; i < 4; i++)
    {
      if (p_flowMaps[i]->t.size() == 0)
        return false;
      if (p_flowMaps[i]->t.back() < p_flowMaps[i]->t.front() + tau_a ||
          p_flowMaps[i]->t.back() < p_flowMaps[i]->t.front() + tau_b)
        return false;
    }

    Vec3r advPos[8]; // particle positions after some advection
    advPos[0] = p_flowMaps[0]->eval_position_at(p_flowMaps[0]->t.front() + tau_a);
    advPos[1] = p_flowMaps[0]->eval_position_at(p_flowMaps[0]->t.front() + tau_b);
    advPos[2] = p_flowMaps[1]->eval_position_at(p_flowMaps[1]->t.front() + tau_b);
    advPos[3] = p_flowMaps[1]->eval_position_at(p_flowMaps[1]->t.front() + tau_a);
    advPos[4] = p_flowMaps[2]->eval_position_at(p_flowMaps[2]->t.front() + tau_a);
    advPos[5] = p_flowMaps[2]->eval_position_at(p_flowMaps[2]->t.front() + tau_b);
    advPos[6] = p_flowMaps[3]->eval_position_at(p_flowMaps[3]->t.front() + tau_b);
    advPos[7] = p_flowMaps[3]->eval_position_at(p_flowMaps[3]->t.front() + tau_a);

    diffVec[0] = point_a - advPos[0];
    diffVec[1] = point_a - advPos[1];
    diffVec[2] = point_b - advPos[2];
    diffVec[3] = point_b - advPos[3];
    diffVec[4] = point_a - advPos[4];
    diffVec[5] = point_a - advPos[5];
    diffVec[6] = point_b - advPos[6];
    diffVec[7] = point_b - advPos[7];

    // check if one of the particles is at the same position at two different
    // times
    real moveDist[8];
    moveDist[0] = (advPos[0] - advPos[1]).norm();
    moveDist[1] = (advPos[2] - advPos[3]).norm();
    moveDist[2] = (advPos[4] - advPos[5]).norm();
    moveDist[3] = (advPos[6] - advPos[7]).norm();
    moveDist[4] = (advPos[0] - advPos[4]).norm();
    moveDist[5] = (advPos[1] - advPos[5]).norm();
    moveDist[6] = (advPos[2] - advPos[6]).norm();
    moveDist[7] = (advPos[3] - advPos[7]).norm();

    if (moveDist[0] < Globals::ZERO || moveDist[1] < Globals::ZERO ||
        moveDist[2] < Globals::ZERO || moveDist[3] < Globals::ZERO ||
        moveDist[4] < Globals::ZERO || moveDist[5] < Globals::ZERO ||
        moveDist[6] < Globals::ZERO || moveDist[7] < Globals::ZERO)
      return false;

    return true;
  }

  //--------------------------------------------------------------------------//
  /**Compute all information that are needed for the RecPoint, based on pos, t0
 * and tau. */
  RecPoint
  HyperLine::createRecPoint(const Vec3r &pos,
                            const real &t0,
                            const real &tau) const
  {
    // if so average the point from the available infos and return it
    FlowMap3D flowMap = mp_flowSampler->sampleFlow(pos, t0, tau);
    Vec3r endPoint = flowMap.y.back();
    Vec3r n(0.0, 0.0, 0.0);
    Vec3r temp(0.0, 0.0, 0.0);
    flowMap.eval_at(t0, temp, &n);
    n.normalize();
    real dist = (endPoint - pos).norm2();
    RecPoint newPoint(pos, n, t0, tau, dist);
    return newPoint;
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//
