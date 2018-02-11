#ifndef OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_EULER_H
#define OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_EULER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_integrator.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Semi-Implicit Euler Integrator Class.
    */
    template< typename Types >
    class Euler : public Integrator<Types>
    {
    public:
      typedef Integrator<Types>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_container  particle_container;
      typedef typename Types::collision_detection  collision_detection;

    public:
      /**
      * Default Constructor.
      */
      Euler(const real_type& timestep, const real_type& restitution)
        : base_type(timestep, restitution)
      {}

      /**
      * Deconstructor.
      */
      ~Euler()
      {}

    public:

      /**
      * Integrate
      *
      * @param timestep  > 0
      */
      void integrate(particle& par) const
      {
        const vector &a = par.force()/par.density();
        vector &x = par.position();
        vector &v = par.velocity();

        // advance velocity
        v += a*base_type::m_dt;

        // move particle (semi-implicit, as v is at time n+1)
        x += v*base_type::m_dt;

        // COLLISION TEST
        if (base_type::m_colisys) {
          typename collision_detection::collision_type coli;
          if (base_type::m_colisys->collision(coli, par)) {
            x = coli.contact();
            const real_type r = base_type::m_r>0?base_type::m_r*coli.penetration()/(base_type::m_dt*(length(v))):0;
            v -= (1+r)*(v*coli.normal())*coli.normal();
            //const real_type l = base_type::m_dt*(length(v));
            //v -= (1.+(l>0?(base_type::m_r*coli.penetration()/l):0))*(v*coli.normal())*coli.normal();
            //v -= (1+base_type::m_r)*(v*coli.normal())*coli.normal();
          }
        }
      }

    }; // End class Euler

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_EULER_H
#endif
