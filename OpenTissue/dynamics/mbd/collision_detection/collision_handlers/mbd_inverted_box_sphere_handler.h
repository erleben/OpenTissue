#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_INVERTED_BOX_SPHERE_HANDLER_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_INVERTED_BOX_SPHERE_HANDLER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

//#include <OpenTissue/collision/collision_sphere_sphere.h>

namespace OpenTissue
{
  namespace mbd
  {
    namespace collision_detection
    {
      template<typename mbd_types>
      struct InvertedBoxSphereHandler
      {
        typedef typename mbd_types::math_policy                   math_policy;

        typedef typename math_policy::real_type        real_type;
        typedef typename math_policy::index_type       size_type;
        typedef typename math_policy::value_traits     value_traits;
        typedef typename math_policy::vector3_type     vector3_type;
        typedef typename math_policy::quaternion_type  quaternion_type;
        typedef typename math_policy::coordsys_type    coordsys_type;

        typedef typename mbd_types::body_type                     body_type;
        typedef typename mbd_types::material_type                 material_type;
        typedef typename mbd_types::contact_container             contact_container;
        typedef typename mbd_types::contact_type                  contact_type;
        typedef typename mbd_types::node_traits                   node_traits;

        typedef OpenTissue::geometry::Sphere<math_policy>      sphere_type;
        typedef OpenTissue::geometry::OBB<math_policy>         box_type;

        typedef typename mbd_types::collision_info_type   collision_info_type;


        static bool test(
             sphere_type & sphere
           , box_type & box
           , collision_info_type & info
           )
        {
          using std::fabs;
          using std::sqrt;

          coordsys_type BtoWCS;
          coordsys_type StoWCS;
          vector3_type r_s;
          vector3_type r_b;
          quaternion_type Q_s;
          quaternion_type Q_b;
          info.get_body_A()->get_position( r_s );
          info.get_body_A()->get_orientation( Q_s );
          info.get_body_B()->get_position( r_b );
          info.get_body_B()->get_orientation( Q_b );

          BtoWCS = coordsys_type( r_b, Q_b );
          StoWCS = coordsys_type( r_s, Q_s );

          //--- Transform sphere center into model frame of box
          coordsys_type StoB;
          StoB = model_update(StoWCS,BtoWCS);
          vector3_type c = sphere.center();
          StoB.xform_point(c);

          //--- get sphere radius (r) and half side lengths (called
          //--- extensions) of the box (a)
          real_type r = sphere.radius();
          vector3_type a = box.ext();
          //--- Cut line from center of box to center of sphere by the box
          //--- faces. This is done in the model frame of the box.
          bool inside = true;
          vector3_type n;
          vector3_type p = c;
          if(c(0)>a(0))
          {
            p(0) = a(0);
            n(0) = value_traits::one();
            inside = false;
          }
          if(c(0)<-a(0))
          {
            p(0) = -a(0);
            n(0) = -value_traits::one();
            inside = false;
          }
          if(c(1)>a(1))
          {
            p(1) = a(1);
            n(1) = value_traits::one();
            inside = false;
          }
          if(c(1)<-a(1))
          {
            p(1) = -a(1);
            n(1) = -value_traits::one();
            inside = false;
          }
          if(c(2)>a(2))
          {
            p(2) = a(2);
            n(2) = value_traits::one();
            inside = false;
          }
          if(c(2)<-a(2))
          {
            p(2) = -a(2);
            n(2) = -value_traits::one();
            inside = false;
          }

          //--- If the the center-line did intersect the box faces or
          //--- if the sphere center lies inside the box then we are in
          //--- the case of being ``inside''.
          if(inside)
          {
            info.get_contacts()->clear();

            //--- We search for the face of the box closest to the center of the sphere.
            //---
            //--- We then set the contact point equal to the deepst point of the sphere (in
            //--- direction of the normal), and the contact normal equal to the normal of
            //--- the closest box face.
            //---
            //--- Penetration depth is simply the distance between the sphere and the
            //--- closest face plus the radius of the sphere.
            size_type cnt =0;
            vector3_type f;
            f(0) = a(0) - fabs(c(0));
            f(1) = a(1) - fabs(c(1));
            f(2) = a(2) - fabs(c(2));

            vector3_type dir[3];
            dir[0] = vector3_type(value_traits::one(),value_traits::zero(),value_traits::zero());
            dir[1] = vector3_type(value_traits::zero(),value_traits::one(),value_traits::zero());
            dir[2] = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one());

            // Find a permuation of axes i,j,k
            //
            //   {i,j,k} -> {a,b,c}
            //
            // such that
            //
            //   f( a ) < f( b ) < f( c )
            //           
            size_type permutation[3] = {0,1,2};
            for(size_type i=1;i<3;++i)
            {
              for(size_type j=i;j>0;--j)
              {
                if(f(permutation[j]) < f(permutation[j-1]))
                {
                  size_type tmp = permutation[j-1];
                  permutation[j-1] = permutation[j];
                  permutation[j] = tmp;
                }
              }
            }

            // Determine the quadrant that the center of the sphere
            // is placed in... that is determine the sign of sphere
            // center position
            int sign[3];
            for(size_type i=0;i<3;++i)
              sign[i] = boost::numeric_cast<int>( (c(i)<value_traits::zero())?-value_traits::one():value_traits::one() );

            //--- Determine how many sides of the box that intersect the sphere
            size_type case_idx = 0;

            for(size_type i=0;i<3;++i)
              if(f(i) <= r)
                ++case_idx;

            vector3_type p[3];
            real_type distance[3];
            vector3_type n[3];

            switch(case_idx)
            {
            case 0:
              return false;
            case 1:
              {
                cnt = 1;
                distance[0] = -(r -f(permutation[0]));
                n[0] = -sign[permutation[0]]*dir[permutation[0]];
                p[0] = c - n[0]*r;
              }
              break;
            case 2:
              {
                for(size_type i=0;i<2;++i)
                {
                  p[i] = c + sign[permutation[i]]*dir[permutation[i]]*r;   //--- farthest point on sphere in direction of i'th closest face
                  n[i] = -sign[permutation[i]]*dir[permutation[i]];
                  distance[i] = -(r-f(permutation[i]));           //--- distance from box to p[i]
                }
                cnt = 2;
              }
              break;
            case 3:
              {
                for(size_type i=0;i<3;++i)
                {
                  p[i] = c + sign[permutation[i]]*dir[permutation[i]]*r;   //--- farthest point on sphere in direction of i'th closest face
                  n[i] = -sign[permutation[i]]*dir[permutation[i]];
                  distance[i] = -(r-f(permutation[i]));           //--- distance from box to p[i]
                }
                cnt = 3;
              }
              break;
            };

            real_type min_distance = value_traits::infinity();
            for(size_type i=0;i<cnt;++i)
            {
              min_distance = (min_distance<distance[i])?min_distance:distance[i];
              if(distance[i]<info.get_envelope())
              {
                contact_type contact;
                BtoWCS.xform_vector(n[i]);
                BtoWCS.xform_point(p[i]);
                contact.init( info.get_body_B(), info.get_body_A(), p[i], n[i], distance[i], info.get_material() );
                info.get_contacts()->push_back(contact);
              }
            }
            return (min_distance <  -info.get_envelope() );
          }
          else
          {
            //--- Sphere center was outside inverted box
            real_type tmp = sqrt(n*n);
            n /= -tmp;
            vector3_type diff = c - p;
            real_type distance = -(length(diff)+r);
            p = p + n*distance;
            contact_type contact;
            BtoWCS.xform_vector(n);
            BtoWCS.xform_point(p);
            contact.init( info.get_body_B(), info.get_body_A(), p, n, distance, info.get_material() );
            info.get_contacts()->push_back(contact);
            return (distance  <  -info.get_envelope() );
          }
          return false;          
        }

        static bool mirrowed_test(
              box_type & box
           ,  sphere_type & sphere
           , collision_info_type & info
           )
        {
          info.flip_bodies();
          return test( sphere, box, info);
        }


      };

    } // namespace collision_detection
  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_COLLISION_HANDLERS_MBD_INVERTED_BOX_SPHERE_HANDLER_H
#endif
