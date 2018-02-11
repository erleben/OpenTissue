#ifndef OPENTISSUE_CORE_GEOMETRY_AABB_UNION_H
#define OPENTISSUE_CORE_GEOMETRY_AABB_UNION_H
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
  namespace geometry
  {
    /**
    * AABB Union.
    * Given two AABBs this method computes a new AABB, which exactly encloses the two given AABB.
    *
    * @param A  The first AABB.
    * @param B  The second AABB.
    *
    * @return   The resulting enclosing AABB.
    */
    template <typename aabb_type>
    aabb_type  aabb_union(aabb_type const & A, aabb_type const & B)
    {
      return aabb_type( min( A.min(), B.min()),   max( A.max(), B.max()) );
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_AABB_UNION_H
#endif
