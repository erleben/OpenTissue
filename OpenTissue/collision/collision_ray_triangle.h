#ifndef OPENTISSUE_COLLISION_COLLISION_RAY_TRIANGLE_H
#define OPENTISSUE_COLLISION_COLLISION_RAY_TRIANGLE_H
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
    * Ray Triangle Collision Test.
    *
    * @param p           origin of ray.
    * @param r           ray direction vector.
    * @param p0          First point of triangle.
    * @param p1          Second point of triangle.
    * @param p2          Third point of triangle.
    * @param t           The ray parameter value at the point of collision (if any).
    *
    *
    * @return            True if ray overlaps triangle, otherwise false.
    */
    template<typename vector3_type,typename real_type>
    bool ray_triangle(
      vector3_type const & p
      , vector3_type const & r
      , vector3_type const & p0
      , vector3_type const & p1
      , vector3_type const & p2
      , real_type & t
      )
    {
      assert(r(0)!=0 || r(1)!=0 || r(2)!=0 || !"ray_triangle(): ray vector was zero!");

      vector3_type  e0 = p1 - p0;   //--- triangle edge
      vector3_type  e1 = p2 - p0;   //--- triangle edge
      vector3_type  n = e0 % e1;    //--- triangle normal

      real_type nr  = n*r;

      if(nr>=0)//--- backfacee culling
        return false;

      t = (n*(p0-p))/nr;  //--- Compute Intersection point between face plane and ray
      if(t<0)//--- Test if intertection point is along the ray direction.
        return false;

      vector3_type pp = p + (r*t);  //--- This is the intersection point.

      //--- Now we want to determine if the intersection point is inside the triangle
      real_type w1,w2,w3;
      OpenTissue::geometry::barycentric_algebraic(p0,p1,p2,pp,w1,w2,w3);//--- hmmm, I wonder why this does not work with p instead of pp?

      real_type delta = 10e-5;
      real_type lower = -delta;
      real_type upper = 1.+delta;
      if(
        (w1>lower)&&(w1<upper)
        &&
        (w2>lower)&&(w2<upper)
        &&
        (w3>lower)&&(w3<upper)
        )
        return true;

      //--- This were the old barycentric coordinate test, it did not work?
      //vector3_type Q  = pp - p0;
      //real_type e00 = e0*e0;
      //real_type e01 = e0*e1;
      //real_type e11 = e1*e1;
      //real_type q0 = e0*Q;
      //real_type q1 = e1*Q;
      //real_type delta = e00*e11 -(e01*e01);
      ////--- Compute barycentric coordinates s0 and s1 of intersection point (this is cramer's rule)
      //real_type sigma0 = (e11*q0 - e01*q1);
      //real_type sigma1 = (e00*q1 - e01*q0);
      ////double s0 = sigma0/delta;
      ////double s1 = sigma1/delta;
      ////--- Test whatever barycentric coordinates are "inside" triangle.
      ////if(s0>=0 && s1>=0 && (s0+s1)<=1)
      ////  return true;
      //if( (sigma0>=-0.0001) && (sigma1>=-0.0001) && ((sigma0+sigma1)<=delta))
      //  return true;
      return false;
    }

    template<typename vector3_type, typename triangle_type>
    bool ray_triangle(vector3_type const & p,vector3_type const &  r, triangle_type const & triangle)

    {
      return ray_triangle(p,r,triangle.p0(),triangle.p1(),triangle.p2());
    }

    template<typename ray_type, typename triangle_type>
    bool ray_triangle(ray_type const & ray, triangle_type const & triangle)

    {
      return ray_triangle(ray.p(),ray.r(),triangle.p0(),triangle.p1(),triangle.p2());
    }

  } //End of namespace collision
} // namespace OpenTissue

//OPENTISSUE_COLLISION_COLLISION_RAY_TRIANGLE_H
#endif
