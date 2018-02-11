#ifndef OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_VERLET_H
#define OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_VERLET_H
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
    * SPH Verlet Integrator Class.
    */
    template< typename Types >
    class Verlet : public Integrator<Types>
    {
    public:
      typedef Integrator<Types>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_container  particle_container;
      typedef typename base_type::collision_detection collision_detection;
    public:
      /**
      * Default Constructor.
      */
      Verlet(const real_type& timestep, const real_type& restitution)
        : base_type(timestep, restitution)
      {}

      /**
      * Deconstructor.
      */
      ~Verlet()
      {}

      /**
      * Fubar dummy assignment operator for the lame compiler!
      */
      Verlet& operator=(Verlet const &) {  return *this;  }

    private:

      /**
      * Integrate
      *
      * @param timestep  > 0
      */
      void integrate(particle& par) const
      {
        vector &x = par.position();
        vector &ox = par.position_old();
        const vector bak = x;

        const vector &a = par.force()/par.density();

        // move particle
        x += 0.98*(x-ox)+a*base_type::m_dt2;
        ox = bak;

        // COLLISION TEST
        if (base_type::m_colisys) {
          typename collision_detection::collision_type coli;
          if (base_type::m_colisys->collision(coli, par)) {

            //vector proj(coli.normal()*coli.penetration());

            // project particle out from obstacle and reflect the velocity
            //x += proj;
            //ox -= (1+m_r)*proj;
            x = coli.contact();
            ox -= (1+base_type::m_r)*((x-ox)*coli.normal())*coli.normal();
            //ox -= ((x-ox)*coli.normal()+base_type::m_r)*coli.normal();
            //ox = x;
            //ox = x-m_r*coli.normal();
          }
        }

        // calculate velocity
        par.velocity() = 0.98*base_type::m_invdt*(x-ox);
      }

    }; // End class Verlet

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_VERLET_H
#endif
