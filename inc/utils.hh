#pragma once

#include <algorithm>
#include <array>
#include <utility>

#include "math.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  namespace utils
  {
    //--------------------------------------------------------------------------//
    /** Helper function to supress warnings due to unused paramters */
    template <class... Types>
    void unusedArgs(const Types... /*args*/) {}
    //--------------------------------------------------------------------------//
    /** Check if the components of two vectors have the same sign.
     * \return true, if all component pairs have the same sign.
     * false, if at least one component pair has different signs */
    template <typename T, unsigned int n>
    bool checkSigns(const VC::math::VecN<T, n> &vec0, const VC::math::VecN<T, n> &vec1)
    {
      for (unsigned int i = 0; i < n; i++)
        if (vec0[i] * vec1[i] < 0.0)
          return false;
      return true;
    }
    //--------------------------------------------------------------------------//
    /// creates an array of size N filled with val of type T
    template <typename T, size_t N, size_t... Is>
    auto make_array(const T &val, std::index_sequence<Is...>)
    {
      return std::array<T, N>{((void)Is, val)...};
    }
    //--------------------------------------------------------------------------//
    /// creates an array of size N filled with val of type T
    template <typename T, size_t N>
    auto make_array(const T &val)
    {
      return make_array<T, N>(val, std::make_index_sequence<N>{});
    }
    //--------------------------------------------------------------------------//
    /// Applies function F to all elements of parameter pack ts
    template <typename F, typename... Ts>
    void for_each(F &&f, Ts &&...ts)
    {
      using discard_t = int[];
      // creates an array filled with zeros. while doing this f gets called with
      // elements of ts
      (void)discard_t{0, ((void)f(std::forward<Ts>(ts)), 0)...};
    }
    //--------------------------------------------------------------------------//
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//