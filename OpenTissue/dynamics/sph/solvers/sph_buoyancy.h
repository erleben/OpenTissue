#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_BUOYANCY_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_BUOYANCY_H
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
    * SPH Buoyancy Class.
    */
    template< typename Types >
    class Buoyancy : Solver< Types, typename Types::vector >
    {
    public:
      typedef Solver<Types, typename Types::vector>  base_type;
      typedef typename base_type::value  value;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      *  Default Constructor.
      */
      Buoyancy(const real_type& buoyancy, const real_type& rest_density, const vector& gravity) : base_type()
        , m_b(buoyancy)
        , m_r0(rest_density)
        , m_g(gravity)
      {
      }

      /**
      * Deconstructor.
      */
      ~Buoyancy()
      {
      }

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      Buoyancy& operator=(const Buoyancy&)
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
        return value(m_g*(m_b*(par.density()-m_r0)));
      }

      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle&) const
      {
        return value(m_g*(m_b*(par.density()-m_r0)));
      }

    private:
      const real_type  m_b;    ///< Buoyancy coefficient.
      const real_type  m_r0;   ///< Rest Density
      const vector  m_g;  ///< Gravitational Acceleration.

    }; // End class Buoyancy

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_BUOYANCY_H
#endif
