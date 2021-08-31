#include "vectorcuboid.hh"

//-------------------------------------------------------------------------//
namespace RS
{
  //-------------------------------------------------------------------------//
  VectorCuboid::VectorCuboid(void)
  {
    for (auto &entry : m_scale)
      entry = 1.0;
  }

  //-------------------------------------------------------------------------//
  const Vec3r *
  VectorCuboid::getVectors(void) const
  {
    return &m_vectors[0];
  }

  //-------------------------------------------------------------------------//
  const real *
  VectorCuboid::getScale(void) const
  {
    return &m_scale[0];
  }

  //-------------------------------------------------------------------------//
  VectorCuboid::VectorCuboid(const Vec3r *p_vec, const real *p_scale)
  {
    for (int i = 0; i < 8; i++)
    {
      m_vectors[i] = *p_vec;
      p_vec++;
    }
    for (int i = 0; i < 3; i++)
    {
      if (p_scale)
      {
        m_scale[i] = *p_scale;
        p_scale++;
      }
      else
        m_scale[i] = 1.0;
    }
  }

  //-------------------------------------------------------------------------//
  VectorCuboid::~VectorCuboid(void) {}

  //-------------------------------------------------------------------------//
  /** Checks if each component contains positive and negative values. If so, the
   * test is passed. This is a necessary property for the existence of singularity.
   * We test not against 0 but against a very small value.
   * \return \c true if the test is passed. */
  bool
  VectorCuboid::passesSignTest(void) const
  {
    real eps = Globals::EPS;
    for (int i = 0; i < 3; i++)
    {
      bool allBelowZero =
          true; // flag indicating all components are smaller, than the threshold
      bool allAboveZero =
          true; // flag indicating all components are bigger, than the threshold
      for (auto vec : m_vectors)
      {
        allBelowZero = allBelowZero && (vec[i] < (1.0 * eps) ? true : false);
        allAboveZero = allAboveZero && (vec[i] > (-1.0 * eps) ? true : false);
      }
      // if one component contains only positive or only
      // negative values, we do not have a critical point
      if (allBelowZero || allAboveZero)
        return false;
    }
    return true;
  }

  //-------------------------------------------------------------------------//
  bool
  VectorCuboid::passesSignTest(const Vec3r *p_intCoef) const
  {
    // create a temporary cube based on the interpolation coordinates
    Vec3r tempVecs[8];
    for (int i = 0; i < 8; i++)
      tempVecs[i] = getInterpolatedVec(p_intCoef[i]);

    // test this cube for critical points
    VectorCuboid tempCube(&tempVecs[0], &m_scale[0]);
    if (!tempCube.passesSignTest())
      return false;

    return true;
  }

  //-------------------------------------------------------------------------//
  /** Checks if the determinant of the Jacobian is equal to 0.
   * The Jacobian is approximated by central differences. It is assumed the cube
   * has sides with unit length.*/
  bool
  VectorCuboid::passesJacobianTest(void) const
  {
    //--- Estimate the Jacobian.
    // a little helper to average vectors
    auto avgVec = [](unsigned int i0,
                     unsigned int i1,
                     unsigned int i2,
                     unsigned int i3,
                     const Vec3r *vec)
    {
      return (vec[i0] + vec[i1] + vec[i2] + vec[i3]) / 4.0;
    };
    Vec3r dx = avgVec(0, 3, 4, 7, &m_vectors[0]) -
               avgVec(1, 2, 5, 6, &m_vectors[0]) / m_scale[0];
    Vec3r dy = avgVec(0, 1, 4, 5, &m_vectors[0]) -
               avgVec(2, 3, 6, 7, &m_vectors[0]) / m_scale[1];
    Vec3r dz = avgVec(0, 1, 2, 3, &m_vectors[0]) -
               avgVec(4, 5, 6, 7, &m_vectors[0]) / m_scale[2];

    return checkDeterminant(dx, dy, dz);
  }

  //-------------------------------------------------------------------------//
  bool
  VectorCuboid::passesJacobianTest(const Vec3r *p_intCoef) const
  {
    //--- Compute scale factor
    // The vectors are scaled by 1/length of the cube
    real scaleX = (p_intCoef[0] - p_intCoef[1]).norm();
    real scaleY = (p_intCoef[0] - p_intCoef[3]).norm();
    real scaleZ = (p_intCoef[0] - p_intCoef[4]).norm();
    scaleX = (scaleX * m_scale[0]);
    scaleY = (scaleY * m_scale[1]);
    scaleZ = (scaleZ * m_scale[2]);

    if (Globals::ZERO > scaleX || Globals::ZERO > scaleY ||
        Globals::ZERO > scaleZ)
      return false;

    //--- Create a temporary cube based on the interpolation coordinates
    Vec3r tempVecs[8];
    for (int i = 0; i < 8; i++)
      tempVecs[i] = getInterpolatedVec(p_intCoef[i]);

    //--- Estimate the Jacobian.
    // a little helper to average vectors
    auto avgVec = [](unsigned int i0,
                     unsigned int i1,
                     unsigned int i2,
                     unsigned int i3,
                     const Vec3r *vec)
    {
      return (vec[i0] + vec[i1] + vec[i2] + vec[i3]) / 4.0;
    };
    Vec3r dx = avgVec(0, 3, 4, 7, &m_vectors[0]) -
               avgVec(1, 2, 5, 6, &m_vectors[0]) / scaleX;
    Vec3r dy = avgVec(0, 1, 4, 5, &m_vectors[0]) -
               avgVec(2, 3, 6, 7, &m_vectors[0]) / scaleY;
    Vec3r dz = avgVec(0, 1, 2, 3, &m_vectors[0]) -
               avgVec(4, 5, 6, 7, &m_vectors[0]) / scaleZ;

    return checkDeterminant(dx, dy, dz);
  }

  //-------------------------------------------------------------------------//
  /** Checks the determinant of the matrix built by (dx,dy,dz) against 0.
   * \return \li true, if abs(det(dx,dy,dz)) > Globals::ZERO
   * \li false, if abs(det(dx, dy, dz)) ? Globals::ZERO */
  bool
  VectorCuboid::checkDeterminant(const Vec3r &dx,
                                 const Vec3r &dy,
                                 const Vec3r &dz) const
  {
    //--- Estimate the determinant of the Jacobian.
    // use the rule of Sarrus
    real det = dx[0] * dy[1] * dz[2] + dy[0] * dz[1] * dx[2] +
               dz[0] * dx[1] * dy[2] - dx[2] * dy[1] * dz[0] -
               dy[2] * dz[1] * dx[0] - dz[2] * dx[1] * dy[0];
    // scale the determinant by the frobenius norm of the matrix
    real fro = sqrt((dx | dx) + (dy | dy) + (dz | dz));
    det /= sqrt(fro);

    //--- Check if the determinant is close to zero
    if (abs(det) <= Globals::DETMIN)
      return false;

    return true;
  }

  //-------------------------------------------------------------------------//
  Vec3r
  VectorCuboid::getInterpolatedVec(const Vec3r &intCoords) const
  {
    Vec3r vec_i00 =
        m_vectors[0] * (1.0 - intCoords[0]) + m_vectors[1] * intCoords[0];
    Vec3r vec_i10 =
        m_vectors[3] * (1.0 - intCoords[0]) + m_vectors[2] * intCoords[0];
    Vec3r vec_i01 =
        m_vectors[4] * (1.0 - intCoords[0]) + m_vectors[5] * intCoords[0];
    Vec3r vec_i11 =
        m_vectors[7] * (1.0 - intCoords[0]) + m_vectors[6] * intCoords[0];

    Vec3r vec_ii0 = vec_i00 * (1.0 - intCoords[1]) + vec_i10 * intCoords[1];
    Vec3r vec_ii1 = vec_i01 * (1.0 - intCoords[1]) + vec_i11 * intCoords[1];

    Vec3r vec_iii = vec_ii0 * (1.0 - intCoords[2]) + vec_ii1 * intCoords[2];

    return vec_iii;
  }
  //-------------------------------------------------------------------------//
}
//-------------------------------------------------------------------------//
