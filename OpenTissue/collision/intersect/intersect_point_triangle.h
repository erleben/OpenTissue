#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_TRIANGLE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_TRIANGLE_H
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
    * Point Triangle Test.
    *
    * Method works by first testing wheter the point lies on the the triangle
    * plane (within treshold) then barycentric coords are tested to veryfy if
    * the point lies inside triangle perimeter.
    *
    * @param p
    * @param triangle
    *
    * @return             If intersection then the reutrn value is true otherwise it is false.
    */
    template<typename vector3_type,typename triangle_type>
    bool point_triangle(vector3_type const & p, triangle_type const & triangle)
    {
      typedef typename triangle_type::math_types   math_types;
      typedef typename math_types::real_type       real_type;


      static const real_type threshold = math::working_precision<real_type>();
      static const real_type lower = -threshold;
      static const real_type upper = 1.+threshold;
      OpenTissue::geometry::Plane<math_types> plane(triangle.p0(),triangle.p1(),triangle.p2());

      real_type  d = plane.signed_distance(p);
      if(d > threshold || d < -threshold)
        return false;
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

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_TRIANGLE_H
#endif
