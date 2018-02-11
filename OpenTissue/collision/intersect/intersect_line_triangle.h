#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_TRIANGLE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_TRIANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/geometry/geometry_barycentric.h>
#include <OpenTissue/core/geometry/geometry_plane.h>

namespace OpenTissue
{

  namespace intersect
  {

    /**
    * Line Triangle Intersection Testing.
    *
    * Ignores coplanar line, method finds intersection point with triangle plane and tests barycentric
    * coords to determine intersection state.
    *
    * @param p0           First coordinate of the line segment
    * @param p1           The second coordinate of the line segment.
    * @param triangle     The triangle.
    * @param p            Holds the intersection point upon return.
    *
    * @return             If intersection then the reutrn value is true otherwise it is false.
    */
    template<typename vector3_type,typename triangle_type>
    bool line_triangle(vector3_type const & p0,vector3_type  const & p1,triangle_type const & triangle,vector3_type & p)
    {
      typedef typename triangle_type::math_types   math_types;
      typedef typename math_types::real_type       real_type;

      static real_type const threshold = math::working_precision<real_type>();
      static real_type const lower = -threshold;
      static real_type const upper = 1.+threshold;

      OpenTissue::geometry::Plane<math_types> plane(triangle.p0(),triangle.p1(),triangle.p2());

      real_type d0 = plane.signed_distance(p0);
      real_type d1 = plane.signed_distance(p1);
      if(d0>=0 && d1>=0)
        return false;
      if(d0<0 && d1<0)
        return false;
      real_type t = d0/(d0-d1);
      p = (p1-p0)*t + p0;
      real_type w1,w2,w3;
      OpenTissue::geometry::barycentric_algebraic(triangle.p0(),triangle.p1(),triangle.p2(),p,w1,w2,w3);
      if(
        (w1>lower)&&(w1<upper)
        &&
        (w2>lower)&&(w2<upper)
        &&
        (w3>lower)&&(w3<upper)
        )
      {
        return true;
      }
      return false;
    }

  } //End of namespace intersect

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_TRIANGLE_H
#endif
