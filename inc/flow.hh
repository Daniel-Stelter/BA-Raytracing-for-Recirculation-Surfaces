#pragma once

// #include "vclibs/base/debug.hh"
// #include "vclibs/base/exception.hh"
// #include "vclibs/base/log.hh"
// #include "vclibs/base/printf.hh"

#include "math.hh"
#include "types.hh"
//#include "vclibs/math/Mat2x2.hh"
//#include "vclibs/math/VecN.hh"
//#include "vclibs/math/rk43.hh"
//#include "vclibs/math/roots.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  template <typename T, unsigned int n>
  class Flow
  {
  public:
    //--------------------------------------------------------------------------//
    Flow<T, n>(const real *domain, const std::string &name = "Flow")
        : m_name(name), m_dimension(n)
    {
      init(domain);
    }
    //--------------------------------------------------------------------------//
    virtual ~Flow() { delete[] mp_domain; };
    //--------------------------------------------------------------------------//
    /// throw `VC::math::ode::OutOfDomain` if `!is_inside(_x)`
    void ensureIsInside(const T &pos) const
    /*throw(VC::math::ode::EvalState)*/ {
      if (!isInside(pos))
      {
        throw VC::math::ode::OutOfDomain;
      }
    }
    //--------------------------------------------------------------------------//
    /// compute flow
    virtual T v(real t, const T &pos) const = 0;
    //--------------------------------------------------------------------------//
    /// Check if pos is inside the defined domain (spatial part of bounding box)
    virtual bool isInside(const T &pos) const
    {
      for (unsigned int i = 0; i < (m_dimension * 2); i += 2)
        if ((pos[i / 2] < mp_domain[i]) || (pos[i / 2] > mp_domain[i + 1]))
          return false;
      return true;
    };
    //--------------------------------------------------------------------------//
    const std::string &getName(void) const { return m_name; }
    //--------------------------------------------------------------------------//
    /** Unified method to set parameters indepentend from the
     * flow itself. The caller has to make sure the correct number
     * of parameters is stored in the array, which is given as parameter.
     * \param real* - pointer to a array/vector with parameters
     * \return true - if the parameters were set correctly */
    virtual bool setParameters(const real *) { return true; }
    //--------------------------------------------------------------------------//
    bool isPeriodic(void) const { return m_periodic; }
    void setPeriodic(bool periodic) { m_periodic = periodic; }
    //--------------------------------------------------------------------------//
    bool isInverted(void) const { return m_inverted; }
    void setInverted(bool inverted) { m_inverted = inverted; }
    //--------------------------------------------------------------------------//
    bool isSteady(void) const { return m_steady; }
    void setSteady(real t)
    {
      m_steady = true;
      m_steady_time = t;
    }
    void setUnsteady(void) { m_steady = false; }
    //--------------------------------------------------------------------------//
  protected:
    //--------------------------------------------------------------------------//
    /** The init function has to correctly set the bounding box */
    virtual void init(const real *bbox)
    {
      if (nullptr != mp_domain)
        delete[] mp_domain;
      mp_domain = new real[(m_dimension + 1) * 2];
      for (unsigned int i = 0; i < (m_dimension + 1) * 2; i++)
        mp_domain[i] = bbox[i];
    }
    //--------------------------------------------------------------------------//
    /** Make the time periodic. The last component of the domain specifies the dimension in time. */
    real clampTime(const real &t0) const
    {
      real time = t0;
      real timeSpan = mp_domain[m_dimension * 2 + 1] - mp_domain[m_dimension * 2];
      assert(timeSpan > 0.0);
      while (time < mp_domain[m_dimension * 2])
        time += timeSpan;
      while (time > mp_domain[m_dimension * 2 + 1])
        time -= timeSpan;

      return time;
    }
    //--------------------------------------------------------------------------//
  protected:
    //--------------------------------------------------------------------------//
    std::string m_name;
    real *mp_domain = nullptr;
    unsigned int m_dimension;
    bool m_periodic = false; // if true, the time component is clamped
    bool m_inverted = false; // if true, the velocity is inverted
    bool m_steady = false;
    real m_steady_time = 0.0;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//

  //-----------------------------------------------------------------------------------------------//
  class Flow2D : public Flow<Vec2r, 2>
  {
  public:
    //--------------------------------------------------------------------------//
    Flow2D(const real *p_domain, const std::string &name)
        : Flow(p_domain, name),
          range_x(Range(mp_domain[0], mp_domain[1])),
          range_y(Range(mp_domain[2], mp_domain[3])),
          range_t(Range(mp_domain[4], mp_domain[5])) {}
    //--------------------------------------------------------------------------//
    virtual ~Flow2D(void) {}
    //--------------------------------------------------------------------------//
    /// compute flow
    virtual Vec2r v(real t, const Vec2r &pos) const override = 0;
    //--------------------------------------------------------------------------//
    const Range range_x;
    const Range range_y;
    const Range range_t;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//

  //-----------------------------------------------------------------------------------------------//
  class Flow3D : public Flow<Vec3r, 3>
  {
  public:
    //--------------------------------------------------------------------------//
    Flow3D(const real *p_domain, const std::string &name)
        : Flow(p_domain, name),
          range_x(Range(mp_domain[0], mp_domain[1])),
          range_y(Range(mp_domain[2], mp_domain[3])),
          range_z(Range(mp_domain[4], mp_domain[5])),
          range_t(Range(mp_domain[6], mp_domain[7])) {}
    //--------------------------------------------------------------------------//
    virtual ~Flow3D(void) {}
    //--------------------------------------------------------------------------//
    /// compute flow
    virtual Vec3r v(real t, const Vec3r &pos) const override = 0;
    //--------------------------------------------------------------------------//
    const Range range_x;
    const Range range_y;
    const Range range_z;
    const Range range_t;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//