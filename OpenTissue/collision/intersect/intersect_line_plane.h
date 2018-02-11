#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_PLANE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_precision.h>

#include <list>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>

namespace OpenTissue
{

  namespace intersect
  {

    /**
    * Line Plane Intersection Test.
    *
    * @param O  The origin of the line.
    * @param D  The destination of the line.
    * @param u  Upon return this argument holds the intersection point.
    * @param t  Upon return this parameter contains a real_type value such that: u = t*D + (1-t)*O
    *
    * @return   If the intersection point is lying between O and 1 then the
    *           return value is true otherwise it is false.
    */
    template<typename vector3_type,typename plane_type,typename real_type>
    bool line_plane(vector3_type const & O,vector3_type const & D,plane_type const & plane, vector3_type & u,real_type & t)
    {
      real_type dO = plane.signed_distance(O);
      real_type dD = plane.signed_distance(D);
      t = dO/(dO-dD);
      u = (D - O)*t + O;
      if(dO>0 && dD>0)
        return false;
      if(dO<0 && dD<0)
        return false;
      return true;
    }


    /**
    * Line Plane Intersection Test.
    *
    * Note that it is trivially assumed that an intersection exist.
    *
    * @param O  The origin of the line.
    * @param D  The destination of the line.
    * @param u  Upon return this argument holds the intersection point.
    *
    * @return   If the intersection point is lying between O and 1 then the
    *           return value is true otherwise it is false.
    */
    template<typename vector3_type,typename plane_type>
    bool line_plane(vector3_type const & O,vector3_type const & D,plane_type const & plane,vector3_type & u)
    {
      typename vector3_type::value_type t;  //--- by standard default constructed integral types are zero!!!
      return line_plane(O,D,plane,u,t);
    }

    /**
    * Line Plane Intersection Test.
    *
    * @param v0
    * @param v1
    *
    * @return         The intersection point (it is trivially assumed that it exist).
    */
    template<typename vector3_type,typename plane_type>
    vector3_type line_plane(vector3_type const & v0,vector3_type const & v1,plane_type const & plane)
    {
      typename vector3_type::value_type s0 = plane.signed_distance(v0);
      typename vector3_type::value_type s1 = plane.signed_distance(v1);
      typename vector3_type::value_type s = s0/(s0-s1);
      return ((v1 - v0)*s + v0);
    }

  } // namespace intersect

} // namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_PLANE_H
#endif
