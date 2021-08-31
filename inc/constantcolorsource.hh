#pragma once

#include "colorsource.hh"
#include "types.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class ConstantColorSource : public ColorSource
  {
    //--------------------------------------------------------------------------//
    color m_color;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    ConstantColorSource(double r, double g, double b) : m_color{r, g, b} {}
    ConstantColorSource(const color &c) : m_color{c} {}
    ConstantColorSource(color &&c) : m_color{std::move(c)} {}
    //--------------------------------------------------------------------------//
    color sample(double /*u*/, double /*v*/) const override { return m_color; }
    //--------------------------------------------------------------------------//
    auto &get_color() { return m_color; }
    const auto &get_color() const { return m_color; }
    //--------------------------------------------------------------------------//
    void set_color(const Vec3r &color) { m_color = color; }
    //--------------------------------------------------------------------------//
    make_clonable(ColorSource, ConstantColorSource);
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//