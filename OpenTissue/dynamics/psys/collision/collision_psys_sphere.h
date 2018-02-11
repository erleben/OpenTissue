#ifndef OPENTISSUE_DYNAMICS_PSYS_COLLISION_COLLISION_PSYS_SPHERE_H
#define OPENTISSUE_DYNAMICS_PSYS_COLLISION_COLLISION_PSYS_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>
#include <cmath>

namespace OpenTissue
{
  namespace psys
  {

    /**
     *
     */
    template<typename particle_system_type, typename sphere_type, typename contact_point_container>
      void collision_psys_sphere(
                           particle_system_type /*const*/ & system
                         , sphere_type const & sphere
                         , contact_point_container & contacts
                         )
    {
      typedef typename particle_system_type::real_type          real_type;
      typedef typename particle_system_type::vector3_type       vector3_type;
      typedef typename contact_point_container::value_type      contact_point_type;      
      typedef typename particle_system_type::particle_iterator  particle_iterator;

      real_type r = sphere.radius();
      real_type r2 = sphere.squared_radius();
      vector3_type center = sphere.center();

      particle_iterator p   = system.particle_begin();
      particle_iterator end = system.particle_end();
      for(;p!=end;++p)
      {
        vector3_type pos  = p->position();
        vector3_type diff = pos - center;
        real_type d2 = diff * diff;
        if(d2<r2)
        {
          contact_point_type cp;
          cp.m_A0       = &(*p);
          cp.m_a0       = real_type();
          cp.m_p        = pos;
          cp.m_n        = unit(diff);
          real_type d   = std::sqrt(d2);
          cp.m_distance = r - d;
          contacts.push_back(cp);
        }
      }
    }

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_COLLISION_COLLISION_PSYS_SPHERE_H
#endif
