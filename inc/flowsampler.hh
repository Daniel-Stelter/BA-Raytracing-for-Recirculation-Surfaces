#pragma once

#include "evaluator.hh"
#include "flow.hh"
#include "timer.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  typedef VC::math::ode::Solution<real, Vec3r> FlowMap3D;
  //--------------------------------------------------------------------------//
  template <typename T, unsigned int n>
  class FlowSampler
  {
  public:
    //--------------------------------------------------------------------------//
    FlowSampler<T, n>(const Flow<T, n> &flow)
        : m_flow(flow), m_eval(Evaluator<T, n>(&m_flow))
    {
      m_odeRK43.options.hmax = 0.01;
      m_odeRK43.options.rsmin = 0.00000005;
    }
    //--------------------------------------------------------------------------//
    ~FlowSampler(void) {}
    //--------------------------------------------------------------------------//
    VC::math::ode::Solution<real, T> sampleFlow(
        const T &position,
        const real &t0,
        const real &tau,
        const int &maxSteps = 0,
        VC::math::ode::EvalState *p_state = nullptr)
    {
      auto sol = VC::math::ode::Solution<real, T>();
      size_t id = TimerHandler::integration_timer().createTimer();
      auto state =
          VC::math::ode::integrate_unsteady<VC::math::ode::RK43<Evaluator<T, n>>>(
              m_odeRK43, &m_eval, position, t0, t0 + tau, &sol, false, maxSteps);
      TimerHandler::integration_timer().deleteTimer(id);
      if (p_state)
      {
        *p_state = state;
      }
      assert(state != VC::math::ode::EvalState::OutOfDomain);
      return sol;
    }
    //--------------------------------------------------------------------------//
    VC::math::ode::EvalState sampleFlow(VC::math::ode::Solution<real, T> *p_sol,
                                        const T &position,
                                        const real &t0,
                                        const real &tau,
                                        const int &maxSteps = 0)
    {
      size_t id = TimerHandler::integration_timer().createTimer();
      auto state =
          VC::math::ode::integrate_unsteady<VC::math::ode::RK43<Evaluator<T, n>>>(
              m_odeRK43, &m_eval, position, t0, t0 + tau, p_sol, false, maxSteps);
      TimerHandler::integration_timer().deleteTimer(id);
      assert(state != VC::math::ode::EvalState::OutOfDomain);
      return state;
    }
    //--------------------------------------------------------------------------//
  private:
    //--------------------------------------------------------------------------//
    VC::math::ode::RK43<Evaluator<T, n>> m_odeRK43;
    const Flow<T, n> &m_flow;
    Evaluator<T, n> m_eval;
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
  typedef FlowSampler<Vec2r, 2> FlowSampler2D;
  typedef FlowSampler<Vec3r, 3> FlowSampler3D;
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//