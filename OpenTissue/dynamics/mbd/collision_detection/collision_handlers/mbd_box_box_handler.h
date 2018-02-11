#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_BOX_BOX_HANDLER_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_BOX_BOX_HANDLER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_obb.h>
#include <OpenTissue/collision/collision_box_box_improved.h>

namespace OpenTissue
{
  namespace mbd
  {
    namespace collision_detection
    {

      template<typename mbd_types>
      struct BoxBoxHandler
      {
        typedef typename mbd_types::collision_info_type   collision_info_type;
        typedef typename mbd_types::math_policy       math_policy;
        typedef typename math_policy::real_type       real_type;
        typedef typename math_policy::index_type      size_type;
        typedef typename math_policy::vector3_type    vector3_type;
        typedef typename math_policy::matrix3x3_type  matrix3x3_type;
        typedef typename math_policy::quaternion_type quaternion_type;
        typedef typename math_policy::coordsys_type   coordsys_type;

        typedef typename mbd_types::body_type                    body_type;
        typedef typename mbd_types::material_type                material_type;
        typedef typename mbd_types::contact_container            contact_container;
        typedef typename mbd_types::contact_type                 contact_type;
        typedef typename mbd_types::node_traits                  node_traits;

        typedef OpenTissue::geometry::OBB<math_policy>        box_type;

        static bool test(
             box_type & boxA
           , box_type & boxB
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
          
          vector3_type p[16];
          vector3_type n;
          real_type distance[16];

          matrix3x3_type RA(AtoWCS.Q());
          matrix3x3_type RB(BtoWCS.Q());
          
          size_t cnt = OpenTissue::collision::box_box_improved(AtoWCS.T(),RA,boxA.ext(),BtoWCS.T(),RB,boxB.ext(),info.get_envelope(),p,n,distance);                 
          
          info.get_contacts()->clear();          
          if(cnt>0)
          {
            for(size_t i=0;i<cnt;++i)
            {
              contact_type contact;
              contact.init( info.get_body_A(), info.get_body_B(), p[i], n, distance[i], info.get_material() );
              info.get_contacts()->push_back(contact);
            }
            return ( distance[0] <  -info.get_envelope() );
          }
          return false;
        }


      };




    } // namespace collision_detection
  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_BOX_BOX_HANDLER_H
#endif
