#ifndef OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_GRAVITY_H
#define OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_GRAVITY_H
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
    * SPH Gravity Class.
    */
    template< typename Types >
    class Gravity : Solver< Types, typename Types::vector >
    {
    public:
      typedef Solver<Types, typename Types::vector>  base_type;
      typedef typename base_type::value              value;
      typedef typename Types::vector                 vector;
      typedef typename Types::particle               particle;
      
      typedef typename Types::particle_cptr_container::const_iterator  particle_cptr_container_citerator;

    public:
      /**
      * Default Constructor.
      * /param g  Gravitational Acceleration
      */
      Gravity(vector const & gravity) 
      : base_type()
      , m_g(gravity)
      {
      }

      ~Gravity(){}

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      Gravity& operator=(Gravity const &)
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
        return value(m_g*par.density());
      }

      /**
      * Apply
      *
      * @param ???   xxx
      */
      value apply(const particle& par, const particle&) const
      {
        return value(m_g*par.density());
      }

    private:
      const vector  m_g;  ///< Gravitational Acceleration.

    }; // End class Gravity

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SOLVERS_SPH_GRAVITY_H
#endif
