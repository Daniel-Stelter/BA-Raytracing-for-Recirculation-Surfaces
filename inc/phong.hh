#pragma once

#include <cmath>

#include "constantcolorsource.hh"
#include "intersectable.hh"
#include "light.hh"
#include "ray.hh"

//--------------------------------------------------------------------------//
namespace RS
{
      //--------------------------------------------------------------------------//
      class Phong
      {
      private:
            //----------------------------------------------------------------------------//
            std::unique_ptr<ColorSource> albedo_color;
            double k_ambient;
            double k_diffus;
            double k_spec;
            double shininess;
            color light_ambient;
            //----------------------------------------------------------------------------//
      public:
            //----------------------------------------------------------------------------//
            Phong(const ColorSource &colorsource,
                  double ambient,
                  double diffus,
                  double specular,
                  double shininess,
                  const Vec3r &light_ambient = {1, 1, 1})
                : albedo_color{colorsource.clone()}, k_ambient{ambient}, k_diffus{diffus},
                  k_spec{specular}, shininess{shininess}, light_ambient{light_ambient} {}
            //----------------------------------------------------------------------------//
            Phong(const color &c,
                  double ambient,
                  double diffus,
                  double specular,
                  double shininess,
                  const Vec3r &light_ambient = {1, 1, 1})
                : Phong{{ConstantColorSource{c}}, ambient, diffus, specular, shininess, light_ambient} {}
            //----------------------------------------------------------------------------//
            ~Phong() = default;
            //----------------------------------------------------------------------------//
            Phong(const Phong &phong)
            {
                  albedo_color = phong.albedo_color->clone();
                  k_ambient = phong.k_ambient;
                  k_diffus = phong.k_diffus;
                  k_spec = phong.k_spec;
                  shininess = phong.shininess;
                  light_ambient = phong.light_ambient;
            }
            //----------------------------------------------------------------------------//
            Phong(Phong &&phong)
            {
                  albedo_color = std::move(phong.albedo_color);
                  k_ambient = phong.k_ambient;
                  k_diffus = phong.k_diffus;
                  k_spec = phong.k_spec;
                  shininess = phong.shininess;
                  light_ambient = phong.light_ambient;
            }
            //----------------------------------------------------------------------------//
            Phong &operator=(const Phong &phong)
            {
                  albedo_color = phong.albedo_color->clone();
                  k_ambient = phong.k_ambient;
                  k_diffus = phong.k_diffus;
                  k_spec = phong.k_spec;
                  shininess = phong.shininess;
                  light_ambient = phong.light_ambient;
                  return *this;
            }
            //----------------------------------------------------------------------------//
            Phong &operator=(Phong &&phong)
            {
                  albedo_color = std::move(phong.albedo_color);
                  k_ambient = phong.k_ambient;
                  k_diffus = phong.k_diffus;
                  k_spec = phong.k_spec;
                  shininess = phong.shininess;
                  light_ambient = phong.light_ambient;
                  return *this;
            }
            //----------------------------------------------------------------------------//
            color sample(const Vec2r &uv) const { return albedo_color->sample(uv); }
            //----------------------------------------------------------------------------//
            Vec3r shade(const Light &light_source, const Intersection &hit) const
            {
                  Vec3r L = light_source.light_direction_to(hit.position).normalize();
                  Vec3r N = hit.normal;
                  Vec3r V = hit.incident_ray.direction();
                  V.normalize();
                  Vec3r R = reflect(L, N);
                  Vec3r light_in = light_source.spectral_intensity();

                  Vec3r i_ambient = light_ambient * k_ambient;

                  double cos_nl = std::max(0.0, (N | -L)); // dot product
                  Vec3r i_diffus = light_in * k_diffus * cos_nl;

                  auto cos_omega = std::max(0.0, (-V | R)); // dot product
                  double spec_power = pow(cos_omega, shininess);
                  Vec3r i_specular = light_in * k_spec * (shininess + 2.0) / (2.0 * M_PI) * spec_power;

                  Vec3r i_all = i_ambient + i_diffus + i_specular;

                  return albedo_color->sample(hit.uv) * i_all;
            }
            //----------------------------------------------------------------------------//
      };
      //----------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//