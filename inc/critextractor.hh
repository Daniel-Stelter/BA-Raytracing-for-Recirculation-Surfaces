#pragma once

#include "mergetool.hh"
#include "vectorcuboid.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  struct CritSearchParams;
  struct CritElements;
  //--------------------------------------------------------------------------//
  /// CritExtractor gets a VectorCuboid and searches critical structures inside.
  class CritExtractor
  {
  public:
    //--------------------------------------------------------------------------//
    /// Get a list with critical structures in (relative) coordinates for a unit cube.
    static CritElements getCritElements(const VectorCuboid &cube);
    //--------------------------------------------------------------------------//
    /** Check if the cube has at least one critical point or structure.
      * \param [in] cube - the VectorCuboid which should be checked
      * \param [out] p_result - optional - if set, the search result is stored in
      * here, which will contain exactly one single element (point/structure). */
    static bool hasCritElements(const VectorCuboid &cube,
                                CritElements *p_result = nullptr);
    //--------------------------------------------------------------------------//
    /** Check if the cube has at least one critical point.
      * If the cube contains also an extended critical structure, it is
      * possible, that isolated points get missed. This is due to the earlier
      * abortion of the search process next to extended structures. */
    static bool hasCritPoint(const VectorCuboid &cube);
    //--------------------------------------------------------------------------//
    /** Check if the cube has at least one critical extended structure. */
    static bool hasCritStructure(const VectorCuboid &cube);
    //--------------------------------------------------------------------------//
    static void setSearchParams(const CritSearchParams &params);
    //--------------------------------------------------------------------------//
    static CritSearchParams getSearchParams(void);
    //--------------------------------------------------------------------------//
  private:
    //--------------------------------------------------------------------------//
    CritExtractor(void);
    //--------------------------------------------------------------------------//
    /** Initialize the interpolation coefficients to the unit cube.
      * \param p_intCoefs - pointer to a Vec3r array of size 8. */
    static void initIntCoefs(Vec3r *p_intCoefs);
    //--------------------------------------------------------------------------//
    static CritElements getCritElements(const VectorCuboid &cube,
                                        const Vec3r *p_intCoef,
                                        const bool &stopAtFirst,
                                        bool *p_passedJacobian = nullptr,
                                        unsigned int *p_step = nullptr);
    //--------------------------------------------------------------------------//
    static std::vector<std::vector<Vec3r>> subDivide(const Vec3r *p_vertices);
    //--------------------------------------------------------------------------//
  private:
    static CritSearchParams m_searchParams;
    //--------------------------------------------------------------------------//
  };

  //--------------------------------------------------------------------------//
  /** Parameters to control the search for critical structures.
    * \li searchPrecision - controls the depth of the recursive search
    * \li jacobiPrecision - controls the depth from which the Jacobian-Test is performed 
    * \li clusterPrecision - distance between two points, to treat them as two different points */
  struct CritSearchParams
  {
    real searchPrecision = 1.0 / pow(2, 40);
    real jacobiPrecision = 1.0 / pow(2, 12);
    real clusterPrecision = 1.0 / pow(2, 38);
    unsigned int maxSteps = static_cast<unsigned int>(pow(8, 7));
  };
  //--------------------------------------------------------------------------//

  //--------------------------------------------------------------------------//
  struct CritElements
  {
    //--------------------------------------------------------------------------//
    std::vector<Vec3r> critPoints;
    std::vector<Vec3r> critStructures;
    //--------------------------------------------------------------------------//
    bool containsCritPoints(void) const { return 0 < critPoints.size(); }
    //--------------------------------------------------------------------------//
    bool containsCritStructures(void) const { return 0 < critStructures.size(); }
    //--------------------------------------------------------------------------//
    unsigned int critPointCount(void) const
    {
      return static_cast<unsigned int>(critPoints.size());
    }
    //--------------------------------------------------------------------------//
    unsigned int critStructureCount(void) const
    {
      return static_cast<unsigned int>(critStructures.size());
    }
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//