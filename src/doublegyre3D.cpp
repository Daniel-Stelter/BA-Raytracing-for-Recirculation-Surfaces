#include "doublegyre3D.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  const real DoubleGyre3D::dg_bbox[8] = {
      0.0, 2.0, 0.0, 1.0, 0.0, 1.0, 0.0, 10.0};

  //-----------------------------------------------------------------------------------------------//
  DoubleGyre3D::DoubleGyre3D(void)
      : Flow3D(&dg_bbox[0], "DoubleGyre3D") {}

  //-----------------------------------------------------------------------------------------------//
  DoubleGyre3D::DoubleGyre3D(const real *bbox)
      : Flow3D(bbox, "DoubleGyre3D") {}

  //-----------------------------------------------------------------------------------------------//
  DoubleGyre3D::DoubleGyre3D(const DoubleGyre3D &other)
      : Flow3D(other.mp_domain, other.m_name), m_A(other.m_A), m_omega(other.m_omega), m_eps(other.m_eps) {}

  //-----------------------------------------------------------------------------------------------//
  DoubleGyre3D &
  DoubleGyre3D::operator=(const DoubleGyre3D &other)
  {
    m_A = other.m_A;
    m_eps = other.m_eps;
    m_omega = other.m_omega;
    Flow3D::init(other.mp_domain);
    return *this;
  }

  //-----------------------------------------------------------------------------------------------//
  DoubleGyre3D::~DoubleGyre3D(void) {}

  //-----------------------------------------------------------------------------------------------//
  void
  DoubleGyre3D::init(const real *bbox)
  {
    Flow3D::init(bbox);
  }

  //-----------------------------------------------------------------------------------------------//
  Vec3r
  DoubleGyre3D::v(real t, const Vec3r &pos) const
  {
    if (isSteady())
      t = m_steady_time;
    if (isPeriodic())
      t = clampTime(t);

    real x = pos[0];
    real y = pos[1];
    real z = pos[2];
    Vec3r v;

    real A = m_A;
    real eps = m_eps;
    real omega = m_omega;

    real a = eps * sin(omega * t);
    real b = 1.0 - 2.0 * a;
    real f = a * x * x + b * x;

    v[0] = -M_PI * A * sin(M_PI * f) * cos(M_PI * y);
    v[1] = M_PI * A * cos(M_PI * f) * sin(M_PI * y) * (2.0 * a * x + b);
    v[2] = omega / M_PI * z * (1.0 - z) * (z - 0.5 - eps * sin(2.0 * omega * t));

    return v;
  }

  //-----------------------------------------------------------------------------------------------//
  bool
  DoubleGyre3D::setParameters(const real *p_params)
  {
    real A = 0.0;
    real omega = 0.0;
    real eps = 0.0;
    try
    {
      A = *p_params;
      p_params++;
      omega = *p_params;
      p_params++;
      eps = *p_params;
    }
    catch (...)
    {
      return false;
    }
    setParameters(A, omega, eps);
    return true;
  }

  //-----------------------------------------------------------------------------------------------//
  void
  DoubleGyre3D::setParameters(const real &A, const real &omega, const real &eps)
  {
    m_A = A;
    m_omega = omega;
    m_eps = eps;
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//
