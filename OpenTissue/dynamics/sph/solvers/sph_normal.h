#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_NORMAL_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_NORMAL_H
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
    * SPH Surface Normal Class.
    */
    template< typename Types, typename KernelPolicy >
    class SurfaceNormal : Solver< Types, typename Types::vector >
    {
    public:
      typedef Solver<Types, typename Types::vector>  base_type;
      typedef KernelPolicy  smoothing_kernel;
      typedef typename base_type::value  value;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      */
      SurfaceNormal() : base_type()
      {
      }

      /**
      * Deconstructor.
      */
      ~SurfaceNormal()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      SurfaceNormal& operator=(const SurfaceNormal&)
      {
        return *this;
      }

    public:
      /**
      * Apply
      *
      * @param ???   xxx
      */
      virtual value apply(const particle& par, particle_cptr_container_citerator begin, particle_cptr_container_citerator end) const
      {
        value res(0);  // reset vector;
        for (particle_cptr_container_citerator p_ = begin; p_ != end; ++p_) {
          const particle* p = *p_;
          res += (p->mass()/p->density())*m_W.gradient(par.position()-p->position());
        }
        return value(res);
      }


      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle& p) const
      {
        return value((p.mass()/p.density())*m_W.gradient(par.position()-p.position()));
      }

    private:
      const smoothing_kernel  m_W;  ///< Smoothing Kernel

    }; // End class SurfaceNormal

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_NORMAL_H
#endif
