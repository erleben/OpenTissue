#ifndef OPENTISSUE_COLLISION_COLLISION_RAY_AABB_H
#define OPENTISSUE_COLLISION_COLLISION_RAY_AABB_H
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
  namespace collision
  {

    /**
    * Ray AABB Collision Test.
    *
    * @param p           Origin of ray.
    * @param r           Ray direction vector.
    * @param min_coord   Minimum coordinate corner of aabb.
    * @param max_coord   Maximum coordinate corner of aabb.
    *
    * @return            True if ray overlaps aabb, otherwise false.
    */
    template<typename vector3_type>
    inline bool ray_aabb(vector3_type const & p,vector3_type const &  r, vector3_type const &  min_coord,vector3_type const &  max_coord)
    {
      using std::fabs;

      typedef typename vector3_type::value_traits   value_traits;

      assert(min_coord(0) <= max_coord(0)  || !"ray_aabb(): aabb was incorrect");
      assert(min_coord(1) <= max_coord(1)  || !"ray_aabb(): aabb was incorrect");
      assert(min_coord(2) <= max_coord(2)  || !"ray_aabb(): aabb was incorrect");
      assert(r(0)!=0 || r(1)!=0 || r(2)!=0 || !"ray_aabb(): ray vector was zero!");

      vector3_type e =  (max_coord - min_coord)*value_traits::half();  //--- Compute extents of box
      vector3_type d =  p - (e + min_coord);                           //--- Compute vector from center of box to origin of ray

      //--- Test if one of three axes of the box is a separation axe
      if (fabs(d(0)) > e(0) && d(0)*r(0) >= 0)  return false;
      if (fabs(d(1)) > e(1) && d(1)*r(1) >= 0)  return false;
      if (fabs(d(2)) > e(2) && d(2)*r(2) >= 0)  return false;

      //--- Test if one of three cross producs of the axes and the ray direction is a separation axe
      vector3_type rXd = cross(r , d);

      if ( fabs(rXd(0)) > ( e(1)*fabs(r(2)) + e(2)*fabs(r(1)) ) )  return false;
      if ( fabs(rXd(1)) > ( e(0)*fabs(r(2)) + e(2)*fabs(r(0)) ) )  return false;
      if ( fabs(rXd(2)) > ( e(0)*fabs(r(1)) + e(1)*fabs(r(0)) ) )  return false;

      //--- No sepration axe exist, we must have a collision
      return true;
    }

    template<typename vector3_type, typename aabb_type>
    inline bool ray_aabb(vector3_type const & p,vector3_type const &  r, aabb_type const & aabb)
    {
      return ray_aabb(p,r,aabb.min_coord(),aabb.max_coord());
    }

    template<typename ray_type, typename aabb_type>
    inline bool ray_aabb(ray_type const & ray, aabb_type const & aabb)
    {
      return ray_aabb(ray.p(),ray.r(),aabb.min_coord(),aabb.max_coord());
    }

  } //End of namespace collision
} // namespace OpenTissue

//OPENTISSUE_COLLISION_COLLISION_RAY_AABB_H
#endif
