#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_AABB_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_AABB_H
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
  namespace intersect
  {

    /**
    * Point AABB intersection test.
    *
    *
    * @param p           The point to be tested.
    * @param min_coord   The minimum coordinate point of the AABB box.
    * @param max_coord   The maximum coordinate point of the AABB box.
    *
    * @return            If the point p lies inside the AABB box then
    *                    the return value is true otherwise it is false.
    */
    template<typename vector3_type>
    bool point_aabb(vector3_type const & p,vector3_type const & min_coord,vector3_type const & max_coord)
    {
      if( min_coord(0) > p(0) )
        return false;
      if( max_coord(0) < p(0) )
        return false;
      if( min_coord(1) > p(1) )
        return false;
      if( max_coord(1) < p(1) )
        return false;
      if( min_coord(2) > p(2) )
        return false;
      if( max_coord(2) < p(2) )
        return false;
      return true;
    }

    template<typename vector3_type,typename aabb_type>
    bool point_aabb(vector3_type const & p,aabb_type const & aabb)
    {
      return point_aabb(p,aabb.min_coord(),aabb.max_coord());
    }

  } // namespace intersect

} // namespace OpenTissue

//OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_AABB_H
#endif
