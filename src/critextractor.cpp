#include "critextractor.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  CritSearchParams CritExtractor::m_searchParams = CritSearchParams();

  //-----------------------------------------------------------------------------------------------//
  void
  CritExtractor::setSearchParams(const CritSearchParams &params)
  {
    m_searchParams = params;
  }

  //-----------------------------------------------------------------------------------------------//
  CritSearchParams
  CritExtractor::getSearchParams(void)
  {
    return m_searchParams;
  }

  //-----------------------------------------------------------------------------------------------//
  void
  CritExtractor::initIntCoefs(Vec3r *p_intCoefs)
  {
    p_intCoefs[0] = Vec3r(0.0, 0.0, 0.0);
    p_intCoefs[1] = Vec3r(1.0, 0.0, 0.0);
    p_intCoefs[2] = Vec3r(1.0, 1.0, 0.0);
    p_intCoefs[3] = Vec3r(0.0, 1.0, 0.0);
    p_intCoefs[4] = Vec3r(0.0, 0.0, 1.0);
    p_intCoefs[5] = Vec3r(1.0, 0.0, 1.0);
    p_intCoefs[6] = Vec3r(1.0, 1.0, 1.0);
    p_intCoefs[7] = Vec3r(0.0, 1.0, 1.0);
  }

  //-----------------------------------------------------------------------------------------------//
  CritElements
  CritExtractor::getCritElements(const VectorCuboid &cube)
  {
    Vec3r intCoefs[8];
    initIntCoefs(&intCoefs[0]);
    CritElements res = getCritElements(cube, &intCoefs[0], false);
    res.critPoints = MergeTool3D::mergeClosePoints(
        res.critPoints, m_searchParams.clusterPrecision);
    return res;
  }

  //-----------------------------------------------------------------------------------------------//
  bool
  CritExtractor::hasCritElements(const VectorCuboid &cube,
                                 CritElements *p_result)
  {
    Vec3r intCoefs[8];
    initIntCoefs(&intCoefs[0]);
    unsigned int steps = 0;
    CritElements res = getCritElements(cube, &intCoefs[0], true, nullptr, &steps);
    if (nullptr != p_result)
      *p_result = res;
    return (res.containsCritPoints() || res.containsCritStructures());
  }

  //-----------------------------------------------------------------------------------------------//
  bool
  CritExtractor::hasCritPoint(const VectorCuboid &cube)
  {
    Vec3r intCoefs[8];
    initIntCoefs(&intCoefs[0]);
    CritElements res = getCritElements(cube, &intCoefs[0], true);
    return (res.containsCritPoints());
  }

  //-----------------------------------------------------------------------------------------------//
  bool
  CritExtractor::hasCritStructure(const VectorCuboid &cube)
  {
    Vec3r intCoefs[8];
    initIntCoefs(&intCoefs[0]);
    CritElements res = getCritElements(cube, &intCoefs[0], true);
    return (res.containsCritStructures());
  }

  //-----------------------------------------------------------------------------------------------//
  CritElements
  CritExtractor::getCritElements(const VectorCuboid &cube,
                                 const Vec3r *p_intCoef,
                                 const bool &stopAtFirst,
                                 bool *p_passedJacobian,
                                 unsigned int *p_step)
  {
    // a little helper in case of nullptr
    bool tempBool = true;
    if (!p_passedJacobian)
      p_passedJacobian = &tempBool;
    unsigned int tempInt = 0;
    if (!p_step)
      p_step = &tempInt;

    //--- initialize some stuff
    CritElements results;
    //--- The search has five criteria which stop the process.
    // Four are checked at the lowest reached recursion depth.
    // stop 1. The maximum steps for testing this cube were reached
    if (*p_step > m_searchParams.maxSteps)
      return results;

    // stop 2. There is no singularity.
    bool passedSignTest = cube.passesSignTest(p_intCoef);
    if (!passedSignTest)
      return results;

    // stop 3. There is a singularity but no point.
    Vec3r midPt = (p_intCoef[0] + p_intCoef[6]) / 2.0; // mid point of cube
    Vec3r diag = (p_intCoef[6] - p_intCoef[0]);        // diagonal of  cube
    Vec3r scaleDiag =
        diag * Vec3r(cube.getScale()); // scaled diagonal according to scale factors
    real minDim =
        min(min(abs(scaleDiag[0]), abs(scaleDiag[1])), abs(scaleDiag[2]));

    *p_passedJacobian = true;
    if (minDim < m_searchParams.jacobiPrecision)
      *p_passedJacobian = cube.passesJacobianTest(p_intCoef);
    if (!(*p_passedJacobian))
    {
      results.critStructures.push_back(midPt);
      return results;
    }
    // stop 4. There is a point singularity.
    if (minDim < m_searchParams.searchPrecision)
    {
      *p_passedJacobian = true;
      results.critPoints.push_back(midPt);
      return results;
    }

    //--- If we came to this point, we have to go to the next recursion level.
    // Determine the interpolation scheme for the next level
    int failJacobi = 0; // count how many Jacobian-Test failed

    vector<vector<Vec3r>> subCubes = subDivide(p_intCoef);
    for (auto subCube : subCubes)
    {
      bool passJacobi = true;
      CritElements subResults = getCritElements(
          cube, &subCube[0], stopAtFirst, &passJacobi, &(++(*p_step)));
      // stop 5. if we only want to find at least one critical structure and we
      // did
      if (stopAtFirst && (subResults.containsCritPoints() ||
                          subResults.containsCritStructures()))
        return subResults;

      // generate a list, with all found points...
      for (auto cp : subResults.critPoints)
        results.critPoints.push_back(cp);
      //... and structures
      for (auto cp : subResults.critStructures)
        results.critStructures.push_back(cp);

      // If the Jacobian Test failed twice, there is an expanded critical
      // structure in this cube -> do not perform further recursion.
      failJacobi += !passJacobi;
      if (failJacobi == 2)
      {
        static int tempCount = 0;
        tempCount++;
        (*p_passedJacobian) = false;
        return results;
      }
    }
    return results;
  }

  //-----------------------------------------------------------------------------------------------//
  std::vector<std::vector<Vec3r>>
  CritExtractor::subDivide(const Vec3r *p_vertices)
  {
    const Vec3r *v = p_vertices;
    vector<Vec3r> subCubeVerts;
    vector<vector<Vec3r>> subCubes;
    for (int i = 0; i < 8; i++)
    {
      vector<Vec3r> subCubeVerts;
      subCubeVerts.resize(8);
      subCubeVerts[0] = (v[i] + v[0]) / 2.0;
      subCubeVerts[1] = (v[i] + v[1]) / 2.0;
      subCubeVerts[2] = (v[i] + v[2]) / 2.0;
      subCubeVerts[3] = (v[i] + v[3]) / 2.0;
      subCubeVerts[4] = (v[i] + v[4]) / 2.0;
      subCubeVerts[5] = (v[i] + v[5]) / 2.0;
      subCubeVerts[6] = (v[i] + v[6]) / 2.0;
      subCubeVerts[7] = (v[i] + v[7]) / 2.0;
      subCubes.push_back(subCubeVerts);
    }
    return subCubes;
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//
