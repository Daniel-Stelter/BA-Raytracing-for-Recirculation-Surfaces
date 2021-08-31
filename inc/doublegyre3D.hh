#pragma once

#include "flow.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class DoubleGyre3D : public Flow3D
  //--------------------------------------------------------------------------//
  {
  public:
    //--------------------------------------------------------------------------//
    /// Standared constructor uses xyzt-domain [0,2]x[0,1]x[0,1]x[0,10]
    DoubleGyre3D(void);
    DoubleGyre3D(const real *bbox);
    DoubleGyre3D(const DoubleGyre3D &other);
    DoubleGyre3D &operator=(const DoubleGyre3D &other);
    //--------------------------------------------------------------------------//
    virtual ~DoubleGyre3D(void);
    //--------------------------------------------------------------------------//
    /// compute flow
    virtual Vec3r v(real t, const Vec3r &pos) const override;
    //--------------------------------------------------------------------------//
    /**Parameters must be stored in the order: A, omega, eps.*/
    bool setParameters(const real *p_params) override;
    void setParameters(const real &A, const real &omega, const real &eps);
    //--------------------------------------------------------------------------//
  protected:
    //--------------------------------------------------------------------------//
    virtual void init(const real *bbox) override;
    //--------------------------------------------------------------------------//
    const static real dg_bbox[8];
    //--------------------------------------------------------------------------//
    // standard parameters for the DoubleGyre3D
    real m_A = 0.1;
    real m_omega = 2.0 * M_PI / 10.0;
    real m_eps = 0.25;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//

  //--------------------------------------------------------------------------//
  class DoubleGyre2D : public DoubleGyre3D
  {
  public:
    //--------------------------------------------------------------------------//
    DoubleGyre2D(void)
        : DoubleGyre3D() {}
    ~DoubleGyre2D(void) {}
    //--------------------------------------------------------------------------//
    virtual Vec3r v(real t, const Vec3r &pos) const override
    {
      if (DoubleGyre3D::isSteady())
        t = DoubleGyre3D::m_steady_time;
      if (DoubleGyre3D::isPeriodic())
        t = DoubleGyre3D::clampTime(t);

      Vec3r pos_2d(pos[0], pos[1], 0.0);
      Vec3r v = DoubleGyre3D::v(t, pos_2d);
      v[2] = 0.0;
      return v;
    }
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//