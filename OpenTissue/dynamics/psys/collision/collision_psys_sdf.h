#ifndef OPENTISSUE_DYNAMICS_PSYS_COLLISION_PSYS_SDF_H
#define OPENTISSUE_DYNAMICS_PSYS_COLLISION_PSYS_SDF_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_gradient_at_point.h>
#include <OpenTissue/core/containers/grid/util/grid_value_at_point.h>
#include <cassert>

namespace OpenTissue
{
  namespace psys
  {
    
    /**
     *
     */
    template<typename particle_system_type, typename sdf_geometry_type, typename contact_point_container>
      void collision_psys_sdf(
                           particle_system_type /*const*/ & system
                         , sdf_geometry_type const & sdf
                         , contact_point_container & contacts
                         )
    {
      typedef typename particle_system_type::real_type          real_type;
      typedef typename particle_system_type::vector3_type       vector3_type;
      typedef typename contact_point_container::value_type      contact_point_type;      
      typedef typename particle_system_type::particle_iterator  particle_iterator;

      vector3_type gradient( 0, 0, 0 );
      vector3_type r;

      particle_iterator p   = system.particle_begin();
      particle_iterator end = system.particle_end();
      for(;p!=end;++p)
      {
        vector3_type pos  = p->position();


        if(!OpenTissue::grid::is_point_inside(sdf.m_phi, pos))
          continue;

        real_type dist = OpenTissue::grid::value_at_point(sdf.m_phi, pos);
        if ( dist == sdf.m_phi.unused() )
          continue;

        if(dist > 0)
          continue;

        vector3_type gradient = OpenTissue::grid::gradient_at_point( sdf.m_phi, pos);
        if ( gradient(0) == sdf.m_phi.unused() )
          continue;

        gradient = unit(gradient);

        contact_point_type cp;
        cp.m_A0       = &(*p);
        cp.m_a0       = real_type();
        cp.m_p        = pos;
        cp.m_n        = unit(gradient);
        cp.m_distance = -dist;
        contacts.push_back(cp);

      }
    }

  } // namespace psys
} // namespace OpenTissue

#endif // OPENTISSUE_DYNAMICS_PSYS_COLLISION_PSYS_SDF_H
