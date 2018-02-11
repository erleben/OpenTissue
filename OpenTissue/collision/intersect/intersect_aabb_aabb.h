#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_AABB_AABB_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_AABB_AABB_H
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
  * AABB Intersection Method.
  *
  * @param Amin   Minimum coordinate point of AABB A.
  * @param Amax   Maximum coordinate point of AABB A.
  * @param Bmin   Minimum coordinate point of AABB B.
  * @param Bmax   Maximum coordinate point of AABB B.
  *
  * @return       If the A and B are intersecting then the return
  *               value is true otherwise it is false.
  */
  template<typename vector3_type>
    bool aabb_aabb(
    vector3_type const & Amin
    , vector3_type const & Amax
    , vector3_type const & Bmin
    , vector3_type const & Bmax
    )
  {
    if(Bmin[0]>Amax[0] || Amin[0]>Bmax[0])
      return false;
    if(Bmin[1]>Amax[1] || Amin[1]>Bmax[1])
      return false;
    if(Bmin[2]>Amax[2] || Amin[2]>Bmax[2])
      return false;
    return true;
  }

  template<typename aabb_type>
    bool aabb_aabb(
    aabb_type const & A
    , aabb_type const & B
    )
  {
    return aabb_aabb(A.min(),A.max(),B.min(),B.max());
  }

} // namespace intersect

} // namespace OpenTissue

//OPENTISSUE_COLLISION_INTERSECT_INTERSECT_AABB_AABB_H
#endif
