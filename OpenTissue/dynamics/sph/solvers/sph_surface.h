#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_SURFACE_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_SURFACE_H
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
    //#define SPH_TENSION_THRESHOLD  0.000001
    //#define SPH_TENSION_THRESHOLD  0.001
    //#define SPH_TENSION_THRESHOLD  4.5
    //#define SPH_TENSION_THRESHOLD  56.25  // 2250/40
    //#define SPH_TENSION_THRESHOLD  12.5  // 500/40

    /**
    * SPH Surface Tension Force Class.
    */
    template< typename Types, typename KernelPolicy >
    class SurfaceForce : Solver< Types, typename Types::vector >
    {
    public:
      typedef Solver<Types, typename Types::vector>  base_type;
      typedef KernelPolicy  smoothing_kernel;
      typedef typename base_type::value  value;
      typedef typename Types::vector  vector;
      typedef typename Types::real_type  real_type;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      */
      SurfaceForce(const real_type& tension = 0, const real_type& threshold = 1) : base_type()
        , m_s(tension)
        , m_l(threshold)
      {
      }

      /**
      * Deconstructor.
      */
      ~SurfaceForce()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      SurfaceForce& operator=(const SurfaceForce& rhs)
      {
        m_s = rhs.m_s;
        m_l = rhs.m_l;
        return *this;
      }

    public:
      real_type& tension()
      {
        return m_s;
      }

      const real_type& tension() const
      {
        return m_s;
      }

      real_type& threshold()
      {
        return m_l;
      }

      const real_type& threshold() const
      {
        return m_l;
      }

      /**
      * Apply
      *
      * @param ???   xxx
      */
      virtual value apply(const particle& par, particle_cptr_container_citerator begin, particle_cptr_container_citerator end) const
      {
        const vector& n = par.normal();
        const real_type chk = n*n;
        if (chk < m_l)
          return value(0);
        real_type tmp = 0;
        for (particle_cptr_container_citerator p_ = begin; p_ != end; ++p_) {
          const particle* p = *p_;
          //        if (&par == p) continue;  // all sph forces apply i != j
          tmp += (p->mass()/p->density())*m_W.laplacian(par.position()-p->position());
        }
        return value((-m_s*tmp)*normalize(n));
      }

    private:
      const smoothing_kernel  m_W;  ///< Smoothing Kernel
      real_type  m_s;      ///< (sigma) tension coefficient.
      real_type  m_l;      ///< tension threshold (squared).

    }; // End class SurfaceForce

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_SURFACE_H
#endif
