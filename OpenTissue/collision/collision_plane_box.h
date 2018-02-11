#ifndef OPENTISSUE_COLLISION_COLLISION_PLANE_BOX_H
#define OPENTISSUE_COLLISION_COLLISION_PLANE_BOX_H
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
  namespace collision
  {

    /**
    * Plane Box Collision Test.
    *
    * @param PtoWCS    Plane model frame to World Coordinate Transformation.
    * @param BtoWCS    Box model frame to World Coordinate Transformation.
    * @param plane     The plane geometry in its model frame.
    * @param box       The box geometry in its model frame.
    * @param envelope  The size of the collision envelope. If clostest point are separted by more than this distance then there is no contact.
    * @param p         Pointer to array of contact points, must have room for at least three vectors.
    * @param n         Upon return this argument holds the contact normal pointing from the plane towards the box.
    * @param distance  Pointer to array of separation (or penetration) distances. Must have room for at least three values.
    *
    * @return          If contacts exist then the return value indicates the number of contacts, if no contacts exist the return valeu is zero.
    */
    template<typename coordsys_type,typename plane_type,typename box_type,typename real_type,typename vector3_type>
    int plane_box(
      coordsys_type const & PtoWCS
      , coordsys_type const & BtoWCS
      , plane_type const & plane
      , box_type const & box
      , real_type const & envelope
      , vector3_type * p
      , vector3_type & n
      , real_type * distance
      )
    {

      typedef typename box_type::matrix3x3_type  matrix3x3_type;

      assert(p);
      assert(distance);

      //--- Transform box center and orientation into model frame of plane geometry
      coordsys_type BtoP = model_update(BtoWCS,PtoWCS);

      vector3_type c = box.center();
      BtoP.xform_point(c);
      matrix3x3_type R = box.orientation();
      BtoP.xform_matrix(R);

      //--- Find closest point between plane and box
      n = plane.n();

      vector3_type i,j,k;
      i(0) = R(0,0);    i(1) = R(1,0);    i(2) = R(2,0);
      j(0) = R(0,1);    j(1) = R(1,1);    j(2) = R(2,1);
      k(0) = R(0,2);    k(1) = R(1,2);    k(2) = R(2,2);

      p[0] = c;
      vector3_type a = box.ext();
      vector3_type delta;//--- keep signs so we know which corner of box that was the clostest point.

      if(n*i>0)
      {
        p[0] -= i*a(0);
        delta(0) = -1;
      }
      else
      {
        p[0] += i*a(0);
        delta(0) = +1;
      }
      if(n*j>0)
      {
        p[0] -= j*a(1);
        delta(1) = -1;
      }
      else
      {
        p[0] += j*a(1);
        delta(1) = +1;
      }
      if(n*k>0)
      {
        p[0] -= k*a(2);
        delta(2) = -1;
      }
      else
      {
        p[0] += k*a(2);
        delta(2) = +1;
      }
      real_type w = plane.w();
      distance[0] = n*p[0] - w;

      //--- If closest points on box is longer away than epsilon, then there
      //--- is no chance for any contacts.
      if(distance[0]>envelope)
        return 0;

      //--- The closest point was within envelope, so it will be the first contact point
      //--- All contacts will have the same normal dictated by the plane, so there is
      //--- no need for storing a normal for each contact point.

      //--- Now find three other corners of the box by going along the edges meeting at the clostest point.
      vector3_type b;
      b(0) = -2.*a(0)*delta(0);
      b(1) = -2.*a(1)*delta(1);
      b(2) = -2.*a(2)*delta(2);
      vector3_type corner[3];
      corner[0] = p[0] + i*b(0);
      corner[1] = p[0] + j*b(1);
      corner[2] = p[0] + k*b(2);

      //--- Pick the two corner points with smallest signed distance, and
      //--- if distance below envelope add them as contacts.
      real_type dist[3];
      dist[0] = corner[0]*n - w;
      dist[1] = corner[1]*n - w;
      dist[2] = corner[2]*n - w;

      //--- Use insertion sort to find contacts with smallest distance
      int permutation[3] = {0,1,2};
      for(int s=1; s<3; ++s)
      {
        for(int t=s; t>0; --t)
        {
          if(dist[permutation[t]] < dist[permutation[t-1]])
          {
            int tmp = permutation[t-1];
            permutation[t-1] = permutation[t];
            permutation[t] = tmp;
          }
        }
      }

      if(dist[permutation[0]]>envelope)
      {
        PtoWCS.xform_point(p[0]);
        PtoWCS.xform_vector(n);
        return 1;
      }
      p[1] = corner[permutation[0]];
      distance[1] = dist[permutation[0]];

      if(dist[permutation[1]]>envelope)
      {
        PtoWCS.xform_point(p[0]);
        PtoWCS.xform_point(p[1]);
        PtoWCS.xform_vector(n);
        return 2;
      }
      p[2] = corner[permutation[1]];
      distance[2] = dist[permutation[1]];

      //---- fourth contact can be generated, but often not necessary
      //p[3] = p[1] + (p[2]-p[0]);
      //distance[3] = n*p[3] - w;
      //PtoWCS.xform_point(p[3]);

      PtoWCS.xform_point(p[0]);
      PtoWCS.xform_point(p[1]);
      PtoWCS.xform_point(p[2]);
      PtoWCS.xform_vector(n);
      return 3;
    }

  } //End of namespace collision

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_PLANE_BOX_H
#endif
