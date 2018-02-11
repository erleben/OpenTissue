#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_SDF_PLANE_HANDLER_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_SDF_PLANE_HANDLER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/sdf/sdf.h>
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/collision/collision_plane_sdf.h>

namespace OpenTissue
{
  namespace mbd
  {
    namespace collision_detection
    {
      template<typename mbd_types>
      struct SDFPlaneHandler
      {
        typedef typename mbd_types::math_policy      math_policy;

        typedef typename math_policy::real_type                real_type;
        typedef typename math_policy::vector3_type             vector3_type;
        typedef typename math_policy::quaternion_type          quaternion_type;
        typedef typename math_policy::coordsys_type            coordsys_type;

        typedef typename mbd_types::body_type                             body_type;
        typedef typename mbd_types::material_type                         material_type;
        typedef typename mbd_types::contact_container                     contact_container;
        typedef typename mbd_types::contact_type                          contact_type;
        typedef typename mbd_types::node_traits                           node_traits;

        typedef OpenTissue::geometry::Plane<math_policy>         plane_type;
        typedef OpenTissue::polymesh::PolyMesh<math_policy>      mesh_type;
        typedef OpenTissue::grid::Grid<float,math_policy>        grid_type;
        typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>   sdf_geometry_type;

        typedef typename mbd_types::collision_info_type   collision_info_type;

       
        static bool test(
             sdf_geometry_type & sdf
           ,  plane_type & plane
           , collision_info_type & info
           )
        {
          coordsys_type BtoWCS;
          coordsys_type AtoWCS;
          vector3_type r_a;
          vector3_type r_b;
          quaternion_type Q_a;
          quaternion_type Q_b;
          info.get_body_A()->get_position( r_a );
          info.get_body_A()->get_orientation( Q_a );
          info.get_body_B()->get_position( r_b );
          info.get_body_B()->get_orientation( Q_b );
          BtoWCS = coordsys_type( r_b, Q_b );
          AtoWCS = coordsys_type( r_a, Q_a );

          info.get_contacts()->clear();

          bool collision = OpenTissue::collision::plane_sdf(BtoWCS, plane, AtoWCS, sdf, *( info.get_contacts() ), info.get_envelope() );
          
          for(typename contact_container::iterator cp = info.get_contacts()->begin();cp!=info.get_contacts()->end();++cp)
          {
            cp->init( info.get_body_B(), info.get_body_A(), cp->m_p, cp->m_n, cp->m_distance, info.get_material() );
          }
          return collision;
        }

        static bool mirrowed_test(
             plane_type & plane
           , sdf_geometry_type & sdf
           , collision_info_type & info
           )
        {
          info.flip_bodies();
          return test( sdf, plane, info);
        }

      };

    } // namespace collision_detection
  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_SDF_PLANE_HANDLER_H
#endif
