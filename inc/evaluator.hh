#pragma once

#include "flow.hh"
#include "utils.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  template <typename T, unsigned int n>
  class Evaluator : public VC::math::ode::base_evaluator<real, T>
  {
  public:
    //--------------------------------------------------------------------------//
    Evaluator<T, n>(const Flow<T, n> *p_flow)
        : mp_flow(p_flow), m_forcedStop(false) {}
    //--------------------------------------------------------------------------//
    virtual ~Evaluator(){};
    //--------------------------------------------------------------------------//
    // Hide parent method
    void dy(const real &t, const T &pos, T &dy)
    {
      mp_flow->ensureIsInside(pos);
      try
      {
        dy = mp_flow->v(t, pos);
      }
      catch (VC::math::ode::EvalState &state)
      {
        if (state == VC::math::ode::EvalState::ForceStop)
          m_forcedStop = true;
        else
          throw state;
      }
    }
    //--------------------------------------------------------------------------//
    // Hide parent method
    void output(real t, const T &pos, const T &dy)
    {
      utils::unusedArgs(t, dy);
      mp_flow->ensureIsInside(pos);
      // VC_DBG_P(_t); VC_DBG_P(_y);  VC_DBG_P(_dy);
      if (m_forcedStop)
      {
        m_forcedStop = false;
        throw VC::math::ode::EvalState::ForceStop;
      }
    }
    //--------------------------------------------------------------------------//
    // virtual void initialize_vector (T&, const T&) {}
    virtual void initialize(real t,
                            const T &pos,
                            VC::math::ode::RK43<Evaluator> *rk)
    {
      utils::unusedArgs(t, rk);
      mp_flow->ensureIsInside(pos);
    }
    //--------------------------------------------------------------------------//
  protected:
    //--------------------------------------------------------------------------//
    const Flow<T, n> *mp_flow;
    bool m_forcedStop;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
  typedef Evaluator<Vec2r, 2> Evaluator2D;
  typedef Evaluator<Vec3r, 3> Evaluator3D;
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//