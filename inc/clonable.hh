#pragma once

#include <memory>

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  template <typename T>
  class Clonable
  {
  public:
    virtual std::unique_ptr<T> clone() const = 0;
  };
  //--------------------------------------------------------------------------//
#define make_clonable(base_t, this_t)            \
  std::unique_ptr<base_t> clone() const override \
  {                                              \
    return std::make_unique<this_t>(*this);      \
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//