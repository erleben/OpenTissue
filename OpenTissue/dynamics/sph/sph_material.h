#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_MATERIAL_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_MATERIAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_idealgas.h>
#include <OpenTissue/core/math/math_constants.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Material Base Class.
    */
    template< typename Types >
    class Material : public IdealGas<typename Types::real_type>
    {
    public:
      typedef IdealGas<typename Types::real_type>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;

    public:
      /**
      * Default Constructor.
      */
      Material()
        : m_name("n/a")
        , m_volume(0)
        , m_density(1000)
        , m_particles(0)
        , m_kernel_particles(10)
        , m_particle_mass(0)
        , m_pressure(1)
        , m_tension(0)
        , m_buoyancy(0)
        , m_viscosity(0)
        , m_restitution(0)
        , m_tempature(293.15)       /// default tempature (20 deg Celsius)
        , m_gas_stiffness(8.3144)
        , m_threshold(1)
        , m_red(0.8)
        , m_green(0.8)
        , m_blue(0.8)
        , m_timestep(0.01)
      {
      }

      /**
      * Deconstructor.
      */
      virtual ~Material()
      {
      }

    public:

      /**
      *
      * @param radius  Upon return holds radius, such that we have X particles within a particle.
      * @param X       Number of partilces to be wihtin radius of a particle.
      */
      virtual real_type radius(const real_type& X) const
      {
        assert(X > 0);
        assert(m_particles);
        //--- Say we want a kernel size, such that at least X other particles are within the kernel distance of a particle!!!
        //---
        //--- The density of particles is
        //---
        //---   sigma  =  N / V
        //---
        //--- If we want X particles inside a kernel then
        //---
        //---
        //---    X = sigma (4/3) pi r^3   =>
        //---
        //---    3 X V  =  4 N pi r^3
        //---
        //---   So
        //---
        //---    r =  pow ( (3 X V) / (4 N pi) , 1/3);
        //---
        const real_type radius = std::pow(((3.*X*m_volume)/(4.*m_particles*math::detail::pi<real_type>())), 1./3.);
#if defined(_DEBUG)
        std::cout << "Suggested radius (with " << X << " particles) = " << radius << "m" << std::endl;
#endif
        return radius;
      }

    public:

      const std::string & name() const { return m_name; }
      size_t const & particles() const { return m_particles; }
      size_t       & particles()       { return m_particles; }

      real_type const & volume() const {return m_volume;}
      
      void volume(real_type const & vol)
      {
        assert(vol > 0);
        assert(m_particles);
        m_volume = vol;
        m_particle_mass = m_density*m_volume/m_particles;
      }

      real_type const & particle_mass() const {return m_particle_mass;}
      
      void particle_mass(real_type const & mass)
      {
        assert(mass > 0);
        m_particle_mass = mass;
        m_volume = m_particles*m_particle_mass/m_density;
      }

      const real_type& threshold() const {return m_threshold;}
      real_type& threshold() {return m_threshold;}

      const real_type& kernel_particles() const {return m_kernel_particles;}

      const real_type& tension() const {return m_tension;}

      const real_type& buoyancy() const {return m_buoyancy;}

      const real_type& viscosity() const {return m_viscosity;}

      const real_type& density() const {return m_density;}

      const real_type& tempature() const {return m_tempature;}

      const real_type& gas_stiffness() const {return m_gas_stiffness;}

      const real_type& timestep() const {return m_timestep;}

      const real_type& restitution() const {return m_restitution;}

      const real_type& red() const {return m_red;}

      const real_type& green() const {return m_green;}

      const real_type& blue() const {return m_blue;}

    protected:

      std::string       m_name;     /// fluid name
      real_type    m_volume;        /// fluid volume
      real_type    m_density;       /// fluid density (limit between expansion/contraction)
      size_t        m_particles;     /// particles to fill fluid volume
      real_type    m_kernel_particles;  /// average particles inside kernel
      real_type    m_particle_mass; /// particle mass
      real_type    m_pressure;      /// fluid pressure in bar
      real_type    m_tension;       /// surface tension
      real_type    m_buoyancy;      /// buoyancy coefficient
      real_type    m_viscosity;     /// fluid viscosity coefficient
      real_type    m_restitution;   /// integration collision bounce-out
      real_type    m_tempature;     /// tempature [SI: Kelvin]
      real_type    m_gas_stiffness; /// strength (stiffness) of the pressure solver
      real_type    m_threshold;     /// surface tension/normal threshold (squared)
      real_type    m_red;           /// highest red tone display color
      real_type    m_green;         /// highest green tone display color
      real_type    m_blue;          /// highest blue tone display color
      real_type    m_timestep;      /// integration timestep

    }; // End class Material

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_MATERIAL_H
#endif
