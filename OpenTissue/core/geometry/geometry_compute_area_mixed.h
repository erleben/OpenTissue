#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_AREA_MIXED_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_AREA_MIXED_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_is_angle_obtuse.h>
#include <OpenTissue/core/geometry/geometry_is_triangle_obtuse.h>
#include <OpenTissue/core/geometry/geometry_compute_triangle_area.h>
#include <OpenTissue/core/geometry/geometry_cot.h>
#include <OpenTissue/core/math/math_precision.h>  // Needed for math::machine_precision

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute the mixed triangle area. 
    * Implicitly assumes that vertex arguments are given in CCW order.
    * This implementation is based on the paper:
    *
    *    Meyer, M., Desbrun, M., Schröder, P., AND Barr, A. H. Discrete Differential Geometry Operators for Triangulated 2-Manifolds, 2002. VisMath.
    *
    * @param p     First vertex of triangle.
    * @param pi    Second vertex of triangle.
    * @param pj    Third vertex of triangle.
    *
    * @return      The mixed area of the triangle.
    */
    template<typename vector_type>
    typename vector_type::value_type compute_area_mixed(vector_type const & p, vector_type const & pi, vector_type const & pj)
    {
      typedef typename vector_type::value_type real_type;

      static const real_type epsilon = math::machine_precision<real_type>();
      static const real_type zero = real_type();  // By standard default integral types are zero

      real_type area = compute_triangle_area(p,pi,pj);

      //--- Triangle too small, area is zero no matter what!
      if(area<epsilon)
        return zero;

      if (is_triangle_obtuse (p,pi,pj)) 
      {
        if (is_angle_obtuse (p,pi,pj))
        {
          return area/2.0;
        }
        return area/4.0;
      } 

      // A_voronnoi
      return ((cot (pi, p, pj) * sqr_length(p - pj) +  cot ( pj, p, pi)* sqr_length(p - pi)) / 8.0);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_AREA_MIXED_H
#endif
