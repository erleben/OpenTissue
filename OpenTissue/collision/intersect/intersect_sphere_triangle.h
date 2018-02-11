#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_SPHERE_TRIANGLE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_SPHERE_TRIANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/intersect/intersect_line_sphere.h>
#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/geometry/geometry_barycentric.h>
#include <OpenTissue/core/geometry/geometry_plane.h>

namespace OpenTissue
{
  namespace intersect
  {


    /**
    * Sphere Triangle Intersection Test.
    *
    *
    * @param p            Center of sphere
    * @param r            Radius of sphere
    * @param triangle
    *
    * @return             If intersection then the reutrn value is true otherwise it is false.
    */
    template<typename sphere_type, typename triangle_type>
    bool sphere_triangle(sphere_type const & sphere, triangle_type const & triangle)
    {
      using std::fabs;

      typedef typename sphere_type::math_types   math_types;
      typedef typename math_types::real_type       real_type;
      typedef typename math_types::vector3_type   vector3_type;

      static real_type const threshold = math::working_precision<real_type>();

      OpenTissue::geometry::Plane<math_types> plane(triangle.p0(),triangle.p1(),triangle.p2());

      vector3_type c = sphere.center();
      real_type    r = sphere.radius();

      real_type  d = plane.signed_distance(c);
      if( fabs(d) > (threshold+r) )  //--- Quick rejection, test if sphere is to far away from triangle-plane
        return false;

      //--- Test if triangle vertices is inside sphere
      if(sphere.contains(triangle.p0()))
        return true;
      if(sphere.contains(triangle.p1()))
        return true;
      if(sphere.contains(triangle.p2()))
        return true;

      //--- Test if triangle edges intersect sphere
      if(line_sphere(triangle.p0(),triangle.p1(),sphere))
        return true;
      if(line_sphere(triangle.p1(),triangle.p2(),sphere))
        return true;
      if(line_sphere(triangle.p2(),triangle.p0(),sphere))
        return true;

      //--- Test if sphere center (projected onto triangle plane) is enclosed by triangle
      static const real_type lower = -threshold;
      static const real_type upper = 1.+threshold;

      real_type w1,w2,w3;
      OpenTissue::geometry::barycentric_algebraic(triangle.p0(),triangle.p1(),triangle.p2(),c,w1,w2,w3);
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

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_SPHERE_TRIANGLE_H
#endif
