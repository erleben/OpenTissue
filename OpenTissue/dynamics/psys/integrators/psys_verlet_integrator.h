#ifndef OPENTISSUE_DYNAMICS_PSYS_INTEGRATOR_POLICIES_PSYS_VERLET_INTEGRATOR_H
#define OPENTISSUE_DYNAMICS_PSYS_INTEGRATOR_POLICIES_PSYS_VERLET_INTEGRATOR_H
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

  namespace psys
  {

    class VerletIntegrator
    {
    public:

      template<typename particle_system_type>
        void integrate(particle_system_type & system, double timestep)
      {
        //--- Compute new position at time t + h
        //---
        //--- Integrating, r'' = a, twice yields
        //---
        //---    r_new = r_cur + v_cur*t + .5*a_cur*t*t   (*1)
        //---
        //--- Now a scond order taylor approximation for r_old around r_cur gives
        //---
        //---    r_old = r_cur + - t v_cur + .5 t*t*a_cur
        //---
        //--- A little mathe-magic yields
        //---
        //---    v_cur  = (r_cur - r_old)/t + .5 t*a_cur   (*2)
        //---
        //--- Substitution of (*2) into (*1) yields
        //---
        //---    r_new = r_cur + ((r_cur - r_old)/t + .5 t*a_cur)*t + .5*a_cur*t*t
        //---
        //--- Cleaning up we have the update rule for Verlet Integration
        //---
        //---    r_new = 2*r_cur - r_old + a_cur*t*t
        //---
        //--- In an efficient implementation t*t can be precomputed and the factor of 2
        //--- can be replaced by an addition by using the trick of:  x = 2*x =>  x += x
        //---
        //---    r_cur += r_cur - r_old * a_cur*t*t
        //---
        typedef typename particle_system_type::real_type          real_type;
        typedef typename particle_system_type::vector3_type       vector3_type;
        typedef typename particle_system_type::particle_iterator  particle_iterator;

        system.compute_forces();
        system.compute_accelerations();

        static real_type zero = boost::numeric_cast<real_type>(0.0);
        static real_type one = boost::numeric_cast<real_type>(1.0);
        real_type dt = boost::numeric_cast < real_type > ( timestep );
        real_type inv_dt  = dt > zero ? one/dt : one;
        real_type dt_sqr  = dt*dt;

        particle_iterator p = system.particle_begin();
        particle_iterator end = system.particle_end();
        for(;p!=end;++p)
        {
          vector3_type r_cur = p->position();
          vector3_type r_old = p->old_position();
          p->old_position() = r_cur;
          vector3_type a_cur = p->acceleration();

          r_cur += r_cur - r_old + a_cur*dt_sqr;
          p->position() = r_cur;

          vector3_type v_cur  = (r_cur - r_old)*(.5*inv_dt); //--- central diff approx to velocity
          p->velocity() = v_cur;
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_INTEGRATOR_POLICIES_PSYS_VERLET_INTEGRATOR_H
#endif
