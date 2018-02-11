#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_SOLVER_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_SOLVER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Compute Base Class.
    */
    template< typename Types, typename Value >
    class Solver
    {
    public:
      typedef          Value  value;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      */
      Solver()
      {
      }

      /**
      * Deconstructor.
      */
      virtual ~Solver()
      {
      }

    public:
      /**
      * Apply a complete solution for par using all pars
      *
      * @param par    The particle at question
      * @param begin  A const iterator to the first particle
      * @param end    A const iterator to the last+1 particle
      */
      virtual value apply(const particle& /*par*/, particle_cptr_container_citerator /*begin*/, particle_cptr_container_citerator /*end*/) const
      {
        return value(0);
      }

      /**
      * Apply a partial solution for par using only a single particle p
      *
      * @param par   The particle at question
      * @param p     A single particle
      */
      virtual value apply(const particle& /*par*/, const particle& /*p*/) const
      {
        return value(0);
      }


    }; // End class Solver

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_SOLVER_H
#endif
