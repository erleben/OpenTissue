#ifndef OPENTISSUE_DYNAMICS_PSYS_INTEGRATOR_POLICIES_PSYS_EULER_INTEGRATOR_H
#define OPENTISSUE_DYNAMICS_PSYS_INTEGRATOR_POLICIES_PSYS_EULER_INTEGRATOR_H
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

    class EulerIntegrator
    {
    public:

      template<typename particle_system_type>
        void integrate(particle_system_type & system, double timestep)
      {
        typedef typename particle_system_type::real_type          real_type;
        typedef typename particle_system_type::vector3_type       vector3_type;
        typedef typename particle_system_type::particle_iterator  particle_iterator;

        system.compute_forces();
        system.compute_accelerations();

        real_type dt = boost::numeric_cast < real_type > ( timestep );

        particle_iterator p = system.particle_begin();
        particle_iterator end = system.particle_end();
        for(;p!=end;++p)
        {
          vector3_type r_cur = p->position();
          p->old_position() = r_cur;
          vector3_type v_cur = p->velocity();
          vector3_type a_cur = p->acceleration();

          r_cur += v_cur*dt;
          v_cur += a_cur*dt;

          p->position() = r_cur;
          p->velocity() = v_cur;
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_INTEGRATOR_POLICIES_PSYS_EULER_INTEGRATOR_H
#endif
