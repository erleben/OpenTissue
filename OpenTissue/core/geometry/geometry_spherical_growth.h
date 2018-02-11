#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SPHERICAL_GROWTH_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SPHERICAL_GROWTH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/geometry/geometry_quadric.h>

namespace OpenTissue
{
  namespace geometry
  {


    /**
    * Spherical Growth.
    * Grows a sphere at point p in the direction of -n, such that the sphere will
    * be the largest empty sphere.
    *
    * NOTE: Supposed to be used with ellipsoid_growing_fit(...).
    *
    * @param p       A surface point on the sphere
    * @param n       Outward surface normal at the point p.
    * @param begin   Iterator to first point
    * @param end     Iterator to one position past the last point
    * @param q       Upon return holds the surface point on the sphere, that caused the largest radius.
    * @param radius  Upon return this argument holds the value of the radius of the inner
    *                largest empty sphere, with p on its surface and normal n. Sphere
    *                center is given by: c = p - radius*n.
    *
    * @return        A quadric representing the grown sphere.
    */
    template<typename real_type, typename vector3_type, typename vector3_iterator>
    Quadric<real_type> spherical_growth(
      vector3_type const & p
      , vector3_type const & n
      , vector3_iterator begin
      , vector3_iterator end
      , real_type & radius
      , vector3_type & q
      )
    {
      radius = math::detail::highest<real_type>();
      for(vector3_iterator x = begin;x!=end;++x)
      {
        vector3_type dx = ((*x) - p);
        real_type tmp = dx*n;
        if( tmp <= 0)
        {
          real_type tst = - (dx*dx) / (2.0*dx*n);
          if(0<tst && tst < radius)
          {
            radius = tst;
            q = (*x);
          }
        }
      }
      assert(radius>0  || !"spherical_growth(): radius was non-positive");
      assert(radius < math::detail::highest<real_type>() || !"spherical_growth(): radius was infinite" );

      vector3_type center = p - n*radius;
      return make_sphere_quadric(radius,center);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SPHERICAL_GROWTH_H
#endif
