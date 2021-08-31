#pragma once

#include "Eigen/Dense"
#include "vclibs/math/VecN.hh"
#include "vclibs/math/rk43.hh"

#include "globals.hh"
#include "types.hh"

//--------------------------------------------------------------------------//
namespace VC
{
  //--------------------------------------------------------------------------//
  namespace math
  {
    //--------------------------------------------------------------------------//
    namespace ode
    {
      //--------------------------------------------------------------------------//
      template <typename T>
      int
      sgn(T val)
      {
        return (T(0) < val) - (val < T(0));
      }
      //--------------------------------------------------------------------------//
      /** Integrate unsteady flow.
       * Wrap, e.g., RK43::integrate(), such that integration continue for CriticalPoint state.
       * Note: the correct way to deal with this case is integration in
       * space-time in a sense that the time t is a component of the vector y! 
       * (For, e.g., sampled flows, this means often t=const.)
       * \tparam ODE integrator, e.g., RK43 */
      template <typename ODE>
      EvalState
      integrate_unsteady(ODE &_ode,
                         typename ODE::evaluator_t *_eval,
                         const typename ODE::vec_t &_y0,
                         typename ODE::real_t _t0,
                         typename ODE::real_t _t1,
                         typename ODE::solution_t *_sol = 0,
                         bool _append = false,
                         int _maxsteps = 0)
      {
        typedef typename ODE::vec_t vec_t;
        typedef typename ODE::real_t real_t;

        assert(_y0.is_finite());

        EvalState state =
            _ode.integrate(_eval, _y0, _t0, _t1, _sol, _append, _maxsteps);

        while (state == CriticalPoint || (_ode.t() == _t0 && _t0 != _t1))
        {
          real_t h = _ode.h();
          real_t normdy = norm2(_ode.dy());
          bool integrate = false;

          real_t t;
          real_t sign_h = _ode.sign_h();

          if (sign_h == real_t(0))
            sign_h = _t0 < _t1 ? real_t(+1) : real_t(-1);

          for (t = _ode.t() + h * sign_h;
               !integrate &&
               ((sign_h > real_t(0) && t < _t1) || (sign_h < real_t(0) && t > _t1));
               t += h * sign_h)
          {

            t = (sign_h > real_t(0)) ? std::min(t, _t1) : std::max(t, _t1);

            vec_t dy;
            try
            {
              _eval->dy(t, _ode.y(), dy);
            }
            catch (EvalState e)
            {
              if (e == OutOfDomain)
                return HitBoundary; // no refinement
              else
                return e;
            }
            if (norm2(dy) > normdy / sqrt(_ode.options.rsmin))
            {
              integrate = true;
              break;
            }
            else
            {
              h = std::min(2 * h, _ode.options.hmax); // better: true bisection
            }
          }

          t = (sign_h > real_t(0)) ? std::min(t, _t1) : std::max(t, _t1);

          if (integrate)
          {
            // VC_DBG_P(t);
            // Note: ||dy||>0 at (t,_y0)
            int maxsteps = _maxsteps > 0 ? _maxsteps - _ode.nsteps() : 0;
            state = _ode.integrate(_eval, _ode.y(), t, _t1, _sol, true, maxsteps);
          }
          else if (_sol != 0)
          {
            _sol->push(_t1, _ode.y(), _ode.dy());
          }

          if (t == _t1)
          {
            state = Success;
            break; // required due to extended condition in "while" (HACK)
          }

          // VC_DBG_P(t);
          // VC_DBG_P(_t1);
          // VC_DBG_P(_ode.y());

          // either we are at t==_t1 or integrate() ran into another CP with t<_t1
        }

        return state;
      }
      //--------------------------------------------------------------------------//
    }
    //--------------------------------------------------------------------------//
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//
namespace RS
{
  /// Calculates the normal of the surface which is spanned by three vectors.
  Vec3r surface_normal(const Vec3r &v1, const Vec3r &v2, const Vec3r &v3);
}
//--------------------------------------------------------------------------//