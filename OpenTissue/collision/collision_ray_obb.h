#ifndef OPENTISSUE_COLLISION_COLLISION_RAY_OBB_H
#define OPENTISSUE_COLLISION_COLLISION_RAY_OBB_H
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
    * Ray OBB Collision Test.
    *
    * @param p           origin of ray.
    * @param r           ray direction vector.
    * @param c           Center of OBB.
    * @param a0          First axis of OBB.
    * @param a1          Second axis of OBB.
    * @param a2          Third axis of OBB.
    * @param e           Half extend of OBB.
    *
    * @return            True if ray overlaps obb, otherwise false.
    */
    template<typename vector3_type>
    bool ray_obb(
      vector3_type const & p
      , vector3_type const &  r
      , vector3_type const &  c
      , vector3_type const &  a0
      , vector3_type const &  a1
      , vector3_type const &  a2
      , vector3_type const &  e
      )
    {
      using std::fabs;

      assert(e(0) >=0 || !"obb was incorrect");
      assert(e(1) >=0 || !"obb was incorrect");
      assert(e(2) >=0 || !"obb was incorrect");
      assert(r(0)!=0 || r(1)!=0 || r(2)!=0 || !"ray vector was zero!");

      vector3_type d =  p - c;               //--- Compute vector from center of box to origin of ray

      if (fabs(a0*d) > e(0) && (a0*d)*(a0*r) >= 0) return false;
      if (fabs(a1*d) > e(1) && (a1*d)*(a1*r) >= 0) return false;
      if (fabs(a2*d) > e(2) && (a2*d)*(a2*r) >= 0) return false;

      //--- Test if one of three cross producs of the axes and the ray direction a separaion axe
      vector3_type rXd = r % d;
      if (fabs(a0*rXd) > (e(1)*fabs(r*a2) + e(2)*fabs(r*a1) )) return false;
      if (fabs(a1*rXd) > (e(0)*fabs(r*a2) + e(2)*fabs(r*a0) )) return false;
      if (fabs(a2*rXd) > (e(0)*fabs(r*a1) + e(1)*fabs(r*a0) )) return false;
      //--- No sepration axe exist, we must have a collision
      return true;
    }

    template<typename vector3_type, typename obb_type>
    bool ray_obb(vector3_type const & p,vector3_type const &  r, obb_type const & obb)

    {
      return ray_obb(p,r,obb.center(),obb.orientation().column(0),obb.orientation().column(1),obb.orientation().column(2),obb.ext());
    }

    template<typename ray_type, typename obb_type>
    bool ray_obb(ray_type const & ray, obb_type const & obb)

    {
      return ray_obb(ray.p(),ray.r(),obb.center(),obb.orientation().column(0),obb.orientation().column(1),obb.orientation().column(2),obb.ext());
    }

  } //End of namespace collision
} // namespace OpenTissue

//OPENTISSUE_COLLISION_COLLISION_RAY_OBB_H
#endif
