#ifndef OPENTISSUE_DYNAMICS_PSYS_COLLISION_PSYS_PLANE_H
#define OPENTISSUE_DYNAMICS_PSYS_COLLISION_PSYS_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace psys
  {

    /**
     *
     */
    template<typename particle_system_type, typename plane_type, typename contact_point_container>
      void collision_psys_plane(
                         particle_system_type /*const*/ & system
                       , plane_type const & plane
                       , contact_point_container & contacts
                       )
    {
      typedef typename particle_system_type::real_type          real_type;
      typedef typename particle_system_type::vector3_type       vector3_type;
      typedef typename contact_point_container::value_type      contact_point_type;      
      typedef typename particle_system_type::particle_iterator  particle_iterator;

      particle_iterator p   = system.particle_begin();
      particle_iterator end = system.particle_end();
      for(;p!=end;++p)
      {
        vector3_type pos  = p->position();
        real_type dst = plane.signed_distance(pos);
        if(dst>0)
          continue;
        contact_point_type cp;
        cp.m_A0       = &(*p);
        cp.m_a0       = real_type();
        cp.m_p        = pos;
        cp.m_n        = unit(plane.n());
        cp.m_distance = -dst;
        contacts.push_back(cp);
      }
    }

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_COLLISION_PSYS_PLANE_H
#endif
