#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_PRESSURE_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_PRESSURE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_solver.h>
#include <OpenTissue/dynamics/sph/sph_particle.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Pressure Class.
    */
    template< typename Types >
    class Pressure : Solver< Types, typename Types::real_type >
    {
    public:
      typedef Solver<Types, typename Types::real_type>  base_type;
      typedef typename base_type::value  value;
      typedef typename Types::real_type  real_type;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      * gas constant = 8.314510   J/(mol K)
      */
      Pressure(const real_type& gas_constant, const real_type& rest_density)
        : base_type()
        , m_k(gas_constant)
        , m_r0(rest_density)
      {
      }

      /**
      * Deconstructor.
      */
      ~Pressure()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      Pressure& operator=(const Pressure&)
      {
        return *this;
      }

    public:
      /**
      * Apply
      *
      * @param ???   xxx
      */
      virtual value apply(const particle& par, particle_cptr_container_citerator, particle_cptr_container_citerator) const
      {
#if 1
        return value(m_k*(par.density()-m_r0));
#elif 1
        using std::sqrt;
        const value diff = par.density()-m_r0;
        return value(diff*(diff>1?sqrt(diff):1));
#else
        using std::pow;
        return value(m_r0*(pow(par.density()/m_r0, m_k)-1));
#endif
      }


      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle&) const
      {
        return value(m_k*(par.density()-m_r0));
      }

    private:
      const real_type  m_k;  ///< Gas Constant.
      const real_type  m_r0;  ///< Rest density (rho0).

    };/* End class Pressure */


    /**
    * SPH Pressure Force Class.
    */
    template< typename Types, typename KernelPolicy >
    class PressureForce : Solver< Types, typename Types::vector >
    {
    public:
      typedef Solver<Types, typename Types::vector>  base_type;
      typedef KernelPolicy  smoothing_kernel;
      typedef typename base_type::value  value;
      typedef typename Types::real_type  real_type;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      */
      PressureForce() : base_type()
      {
      }

      /**
      * Deconstructor.
      */
      ~PressureForce()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      PressureForce& operator=(const PressureForce&)
      {
        return *this;
      }

    public:
      /**
      * Symmetric Pressure Force as suggested Müller et al.
      */
#if 0
      value apply(const particle& par, const particle_cptr_container& pars) const
      {
        value res(0);
        typename particle_cptr_container::const_iterator end = pars.end();
        for (typename particle_cptr_container::const_iterator p_ = pars.begin(); p_ != end; ++p_) {
          const particle* p = *p_;
          if (&par == p) continue;  // all sph forces apply i != j
          res += p->mass()*((par.pressure()+p->pressure())/(2*p->density()))*m_W.gradient(par.position()-p->position());
        }
        return value(-1*res);
      }
#else
      /**
      * Symmetric Pressure Force commonly used by SPH applications; Desbrun et al., etc.
      */
      virtual value apply(const particle& par, particle_cptr_container_citerator begin, particle_cptr_container_citerator end) const
      {
        value res(0);
        const real_type P_rho2 = par.pressure()/(par.density()*par.density());
        for (particle_cptr_container_citerator p_ = begin; p_ != end; ++p_) {
          const particle* p = *p_;
          if (&par == p) continue;  // all sph forces apply i != j
          res += p->mass()*(P_rho2+(p->pressure()/(p->density()*p->density())))*m_W.gradient(par.position()-p->position());
        }
        return value(-par.density()*res);
      }
#endif

      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle& p) const
      {
        return value(-p.mass()*((par.pressure()+p.pressure())/(2*p.density()))*m_W.gradient(par.position()-p.position()));
      }

    private:
      const smoothing_kernel  m_W;  ///< Smoothing Kernel

    }; // End class PressureForce

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_PRESSURE_H
#endif
