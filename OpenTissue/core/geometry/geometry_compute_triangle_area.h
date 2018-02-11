#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TRIANGLE_AREA_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TRIANGLE_AREA_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>   // Needed for std::sqrt and std::fabs

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute Triangle Area.
    *
    * @param p     First vertex of triangle.
    * @param pi    Second vertex of triangle.
    * @param pj    Third vertex of triangle.
    *
    * @return
    */
    template<typename vector_type>
    typename vector_type::value_type compute_triangle_area(vector_type const & p,vector_type const & pi,vector_type const & pj)  
    {
      using std::sqrt;

      typedef typename vector_type::value_type real_type;

      vector_type u = pi-p;
      vector_type v = pj-p;

      // Specialized 3D case
      vector_type n = u%v;
      real_type area = sqrt(n*n)*.5;  // TODO: maybe boost::numeric_cast???

      // General nD formulae...
      //real_type dot = u*v;
      //real_type area = sqrt( (u*u)*(v*v) - dot*dot)*.5; // TODO: maybe boost::numeric_cast???

      return area;
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TRIANGLE_AREA_H
#endif
