#pragma once

#include "globals.hh"
#include "types.hh"
#include "utils.hh"
#include <vector>

//--------------------------------------------------------------------------//
namespace RS
{

  /** This class represents a cube, that gets vectors as its vertices.
   * These vectors represent samples of a vectorfield, i.e. velocity
   * vectors. The cube can be used to find critical points of the
   * vectorfield which are located inside the cube. The samples must
   * have the following order, to be represented correctly.
     7---------6        011-------111
    /|        /|        /|        /|
   / |       / |       / |       / |
  3---------2  |     010-------110 |
  |  |      |  |      |  |      |  |        y
  |  4------|--5      | 001-----|-101       |  z
  | /       | /       | /       | /         | /
  |/        |/        |/        |/          |/
  0---------1        000-------100          o----x
*/
  class VectorCuboid
  {
  public:
    //--------------------------------------------------------------------------//
    VectorCuboid(const Vec3r *p_vec, const real *p_scale = nullptr);
    //--------------------------------------------------------------------------//
    virtual ~VectorCuboid(void);
    //--------------------------------------------------------------------------//
    Vec3r getInterpolatedVec(const Vec3r &intCoords) const;
    //--------------------------------------------------------------------------//
    /** Checks if a cube contains a critical point based on component signs. */
    virtual bool passesSignTest(void) const;
    virtual bool passesSignTest(const Vec3r *p_intCoef) const;
    virtual bool passesJacobianTest(void) const;
    virtual bool passesJacobianTest(const Vec3r *p_intCoef) const;
    //--------------------------------------------------------------------------//
    const Vec3r *getVectors(void) const;
    const real *getScale(void) const;
    //--------------------------------------------------------------------------//
  protected:
    //--------------------------------------------------------------------------//
    VectorCuboid(void);
    //--------------------------------------------------------------------------//
    bool checkDeterminant(const Vec3r &dx,
                          const Vec3r &dy,
                          const Vec3r &dz) const;
    //--------------------------------------------------------------------------//
    Vec3r m_vectors[8]; // velocity vectors at the cuboid corners
    real m_scale[3];
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//