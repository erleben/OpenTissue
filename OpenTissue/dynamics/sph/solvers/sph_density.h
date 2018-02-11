#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_DENSITY_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_DENSITY_H
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
    * SPH Density Class.
    */
    template< typename Types, typename KernelPolicy >
    class Density : Solver< Types, typename Types::real_type >
    {
    public:
      typedef Solver<Types, typename Types::real_type>  base_type;
      typedef KernelPolicy  smoothing_kernel;
      typedef typename base_type::value  value;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      */
      Density() : base_type()
      {
      }

      /**
      * Deconstructor.
      */
      ~Density()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      Density& operator=(const Density&)
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
        value res(0);
#if 0
        static long last = -1;
        long cnt = 0;
        typename particle_cptr_container::const_iterator end = pars.end();
        for (typename particle_cptr_container::const_iterator p_ = pars.begin(); p_ != end; ++p_) {
          const particle* p = *p_;
          cnt += m_W.checkRange(par.position()-p->position())?1:0;
          res += p->mass()*m_W.evaluate(par.position()-p->position());
        }
        if (last >= 0) {
          if (last != cnt)
            std::cout << last << " vs " << cnt << "/" << pars.size() << std::endl;
          last = -1;
        }
        else
          last = cnt;
#else
        for (particle_cptr_container_citerator p_ = begin; p_ != end; ++p_) {
          const particle* p = *p_;
          res += p->mass()*m_W.evaluate(par.position()-p->position());
        }
#endif
        return value(res);
      }


      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle& p) const
      {
        return value(p.mass()*m_W.evaluate(par.position()-p.position()));
      }

    private:
      const smoothing_kernel  m_W;  ///< Smoothing Kernel

    }; // End class Density

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_DENSITY_H
#endif
