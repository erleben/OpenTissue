#ifndef OPENTISSUE_COLLISION_COLLISION_SPHERE_SPHERE_H
#define OPENTISSUE_COLLISION_COLLISION_SPHERE_SPHERE_H
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
    * Sphere Sphere Collision Test.
    *
    * @param cA         Center of sphere A  in world coordinate system.
    * @param rA         Radius of sphere A.
    * @param cB         Center of sphere B  in world coordinate system.
    * @param rB         Radius of sphere B.
    * @param envelope   The collision envelope, if closest points are separated more than this distance, then no contact is generated.
    * @param p          Upon return this argument holds the point of contact in WCS.
    * @param n          Upon return this argument holds the normal of the contact in WCS. Poiting from sphere A towards sphere B.
    * @param distance   Upon return this argument holds separation (or penetration) distance of the contact. Think of it as an overlap measure along the contact normal.
    *
    * @return     If a contact exist between the spheres, then the return value is true otherwise it is false.
    */
    template< typename vector3_type, typename real_type>
    bool sphere_sphere(
      vector3_type const & cA
      , real_type const & rA
      , vector3_type const & cB
      , real_type const & rB
      , real_type const & envelope
      , vector3_type & p
      , vector3_type & n
      , real_type & distance
      )
    {
      //--- Make a vector between the two sphere centers.
      vector3_type diff = cB - cA;
      real_type radius_sum = rA + rB;

      //--- If the distance between the two center spheres are geather than
      //--- the sum of their radius' then there can be no contact between
      //--- the two spheres.
      //--- The tests is performed using squared distances to avoid the
      //--- square root in the distance computations.
      real_type squared_radius_sum_plus_envelope = radius_sum + envelope;
      squared_radius_sum_plus_envelope *= squared_radius_sum_plus_envelope;
      real_type squared_distance = diff*diff;
      if(squared_distance > squared_radius_sum_plus_envelope)
        return false;

      //--- The contact normal is simply the normalized vector between
      //--- the sphere centers.
      //real_type squared_radius_sum = radius_sum*radius_sum;
      n = diff;
      real_type length = sqrt(squared_distance);
      n /= length;

      //--- The contact point is kind of a midpoint weighted by the radius of the
      //--- spheres. That is the contact point, p, is computed  according to the
      //--- proportionality equation
      //---
      //---  |p-cA| /|p-cB| = rA / rB
      //---
      //--- We know
      //---
      //---   dA = |p-cA|
      //---   dB = |p-cB|
      //---    d = |cB-cA| = dA + dB
      //---
      //---        dA = dB (rA/rB)
      //---   dB + dA = dB + dB (rA/rB)   /*  add dB to both sides */
      //---         d = dB(1+rA/rB)       /*  use d=dA+dB          */
      //---
      //---        dB = d/(1+rA/rB)
      //---      d-dA = d/(1+rA/rB)       /*  use d=dA+dB          */
      //---        dA = d - d/(1+rA/rB)
      //---
      real_type dB = (length / ( (rA/rB) + 1.0));
      real_type dA = length - dB;
      distance = length - radius_sum;
      p = n*dA + cA;
      return true;
    }

  } //End of namespace collision

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_SPHERE_SPHERE_H
#endif
