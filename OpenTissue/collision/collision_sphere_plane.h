#ifndef OPENTISSUE_COLLISION_COLLISION_SPHERE_PLANE_H
#define OPENTISSUE_COLLISION_COLLISION_SPHERE_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/math/math_precision.h>

namespace OpenTissue
{
  namespace collision
  {

    /**
    * Sphere Plane Collision Test.
    *
    * @param PtoWCS     Plane to world coordinate transformation.
    * @param StoWCS     Sphere to world coordinate transformation.
    * @param plane      The plane geometry.
    * @param sphere     The sphere geometry.
    * @param envelope   The collision envelope, if closest points are separated more than this distance, then no contact is generated.
    * @param p          Upon return this argument holds the point of contact in WCS.
    * @param n          Upon return this argument holds the normal of the contact in WCS. Pointing from Plane towards sphere.
    * @param distance   Upon return this argument holds separation (or penetration) distance of the contact. Think of it as an overlap measure along the contact normal.
    *
    * @return     If a contact exist between the sphere and the plane, then the return value is true otherwise it is false.
    */
    template<typename coordsys_type,typename sphere_type,typename plane_type,typename real_type,typename vector3_type>
    bool sphere_plane(
      coordsys_type const & StoWCS
      , coordsys_type const & PtoWCS
      , sphere_type const & sphere
      , plane_type const & plane
      , real_type const & envelope
      , vector3_type & p
      , vector3_type & n
      , real_type & distance
      )
    {
      //--- Compute the coordinate transformation, which transforms from
      //--- the model frame of the sphere (S) into the model frame of the
      //--- plane (P).
      coordsys_type StoP = model_update(StoWCS,PtoWCS);

      //--- Retrieve information about the sphere, center (p) and radius (r).
      p = sphere.center();
      real_type r = sphere.radius();

      //--- Transform sphere center into model frame of plane
      StoP.xform_point(p);

      //--- If sphere center is longer away from to the plane
      //--- than its radius (plus the envelope) then there can
      //--- be no contact.
      distance = plane.signed_distance(p);
      if(distance>(envelope+r))
        return false;

      //--- The contact normal must be equal to the plane normal
      n = plane.n();

      //--- The contact point is computed as the projection of
      //--- the sphere center onto the plane.
      p = p - n*distance;

      //--- The penetration distance, must be equal to how deep the sphere
      //--- have sunken down into the plane, so we simply subtract the size
      //--- of the radius from the distance of the center over the plane.
      distance -= r;

      //--- Finally convert contact point and contact normal into world
      //--- coordinate system.
      PtoWCS.xform_point(p);
      PtoWCS.xform_vector(n);
      return true;
    }

  } //End of namespace collision
} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_SPHERE_PLANE_H
#endif
