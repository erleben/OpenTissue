#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_VISCOSITY_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_VISCOSITY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_solver.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Viscosity Force Class.
    */
    template< typename Types, typename KernelPolicy >
    class ViscosityForce : Solver< Types, typename Types::vector >
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
      ViscosityForce(const real_type& viscosity = 0) : base_type()
      {
        setViscosity(viscosity);
      }

      /**
      * Deconstructor.
      */
      ~ViscosityForce()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      ViscosityForce& operator=(const ViscosityForce& rhs)
      {
        m_m = rhs.m_m;
        return *this;
      }

    public:
      void setViscosity(const real_type& coefficient)
      {
        m_m = coefficient;
      }

      /**
      * Apply
      *
      * @param ???   xxx
      */
      virtual value apply(const particle& par, particle_cptr_container_citerator begin, particle_cptr_container_citerator end) const
      {
#if 1
        value res(0);  // reset vector;
        for (particle_cptr_container_citerator p_ = begin; p_ != end; ++p_) {
          const particle* p = *p_;
          if (&par == p) continue;  // all sph forces apply i != j
          res += p->mass()*((p->velocity()-par.velocity())/p->density())*m_W.laplacian(par.position()-p->position());
        }
        return value(m_m*res);
#else
        value res(0);  // reset vector;
        for (particle_cptr_container_citerator p_ = begin; p_ != end; ++p_) {
          const particle* p = *p_;
          if (&par == p) continue;  // all sph forces apply i != j
          res += p->mass()*(p->velocity()-par.velocity())*m_W.laplacian(par.position()-p->position());
        }
        return value(m_m*res/par.density());
#endif
      }


      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle& p) const
      {
        return value(m_m*(p.mass()*((p.velocity()-par.velocity())/p.density())*m_W.laplacian(par.position()-p.position())));
      }

    private:
      const smoothing_kernel  m_W;  ///< Smoothing Kernel
      real_type  m_m;      ///< (mu) viscosity coefficient.

    }; // End class ViscosityForce

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_VISCOSITY_H
#endif
