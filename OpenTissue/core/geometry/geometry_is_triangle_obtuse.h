#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_IS_TRIANGLE_OBTUSE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_IS_TRIANGLE_OBTUSE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_is_angle_obtuse.h>

namespace OpenTissue
{
  namespace geometry
  {
    /**
    * Obtuse Triangle Testing.
    * If any corner of an triangle is obtuse then the triangle is considered to be obtuse.
    *
    * @param p     First vertex of triangle.
    * @param pi    Second vertex of triangle.
    * @param pj    Third vertex of triangle.
    *
    * @return      If triangle is obtuse then the return value is true otherwise it is false.
    */
    template<typename vector_type>
    bool is_triangle_obtuse(vector_type const & p,vector_type const & pi,vector_type const & pj)
    {
      return(    is_angle_obtuse(p,pi,pj) 
        || is_angle_obtuse(pi,pj,p)
        || is_angle_obtuse(pj,p,pi) 
        );
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_IS_TRIANGLE_OBTUSE_H
#endif
