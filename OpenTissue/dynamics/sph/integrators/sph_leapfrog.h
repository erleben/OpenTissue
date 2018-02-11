#ifndef OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_LEAP_FROG_H
#define OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_LEAP_FROG_H
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
    * SPH Leap-Frog Integrator Class.
    *
    * init:        v^(-)  = v^0 -  dt a^0
    *
    * integration:
    *              v^(t+) = v^(t-) + dt a^t
    *              r^t     = r^(t-1) + dt v^(t+)
    *
    * Use Midpoint approximation:    v^t = (v^(t-)+ v^(t+))/2
    *
    * http://bioportal.weizmann.ac.il/course/3dbioinfo/13_Molecular_Dynamics.pdf
    *
    */
    template< typename Types >
    class LeapFrog : public Integrator<Types>
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
      LeapFrog(const real_type& timestep, const real_type& restitution)
        : base_type(timestep, restitution)
      {}

      /**
      * Deconstructor.
      */
      ~LeapFrog()
      {}

    public:

      /**
      * Initialize
      */
      virtual void initialize(particle& par) const
      {
        vector &a0 = par.acceleration();
        vector &v_apx = par.velocity();
        vector &v0 = par.position_old();

        // compute a at time 0
        a0 = par.force()/par.density();

        // compute v at time -
        v0 = v_apx - .5*base_type::m_dt*a0;
      }

      /**
      * Integrate
      *
      * @param timestep  > 0
      */
      void integrate(particle& par) const
      {
        vector &a = par.acceleration();  // a at time t
        vector &v_apx = par.velocity();  // v at time t
        vector &v = par.position_old();  // v at time t -
        vector v_old = v;                // v at time t -
        vector &x = par.position();      // x at time t

        // compute a at time t
        a = par.force()/par.density();

        // compute v at time n +
        v += base_type::m_dt*a;

        // compute x at time n + 1, using v at time n +
        x += base_type::m_dt*v;

        // COLLISION
        if (base_type::m_colisys) {
          typename collision_detection::collision_type coli;
          if (base_type::m_colisys->collision(coli, par)) {
            x = coli.contact();
            const real_type r = base_type::m_r>0?base_type::m_r*coli.penetration()/(base_type::m_dt*(length(v))):0;
            v -= (1+r)*(v*coli.normal())*coli.normal();
            //          v_apx += coli.normal()*(v_apx*coli.normal());
            //          v -= (1+base_type::m_r)*(v*coli.normal())*coli.normal();
            //          v -= (v*coli.normal())*coli.normal();
            //          real_type l = sqr_length(base_type::m_dt*v);
            //          if (l>0) l = 1./l;
            //          v -= (1.+coli.penetration()*coli.penetration()*l)*(v*coli.normal())*coli.normal();
            //v -= ((1+m_r)*(v*n))*n;
            //v_apx -= ((1+m_r)*(v_apx*n))*n;
          }
        }

        // estimate velocity at time t
        v_apx = .5*(v_old+v);

      }
    }; // End class LeapFrog

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_INTEGRATORS_SPH_LEAP_FROG_H
#endif
