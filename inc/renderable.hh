#pragma once

#include "intersectable.hh"
#include "phong.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    //----------------------------------------------------------------------------//
    class Renderable : public Intersectable,
                       public Clonable<Renderable>
    {
        //----------------------------------------------------------------------------//
        Phong m_phong;
        //----------------------------------------------------------------------------//
    public:
        //----------------------------------------------------------------------------//
        Renderable(const Phong &phong) : m_phong{phong} {};
        //----------------------------------------------------------------------------//
        /// Destructor
        virtual ~Renderable() = default;
        //----------------------------------------------------------------------------//
        /// Copy contructor
        Renderable(const Renderable &other)
            : m_phong{other.m_phong} {}
        //----------------------------------------------------------------------------//
        /// Move constructor
        Renderable(Renderable &&other) noexcept
            : m_phong{std::move(other.m_phong)} {}
        //----------------------------------------------------------------------------//
        /// Copy assignment operator
        Renderable &operator=(const Renderable &other)
        {
            m_phong = other.m_phong;
            return *this;
        }
        //----------------------------------------------------------------------------//
        /// Move assignment operator
        Renderable &operator=(Renderable &&other) noexcept
        {
            m_phong = std::move(other.m_phong);
            return *this;
        }
        //----------------------------------------------------------------------------//
        color shade(const Light &l, const Intersection &i) const
        {
            return m_phong.shade(l, i);
        }
        //----------------------------------------------------------------------------//
        color sample(const Vec2r &uv) const
        {
            return m_phong.sample(uv);
        }
        //----------------------------------------------------------------------------//
    };
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//