#ifndef OPENTISSUE_COLLISION_COLLISION_SDF_SDF_H
#define OPENTISSUE_COLLISION_COLLISION_SDF_SDF_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_single_collision_query.h>
#include <OpenTissue/collision/sdf/sdf_collision_policy.h>

namespace OpenTissue
{
  namespace collision
  {

    /**
    * Signed Distance Field Geometry Collision Query.
    *
    * @param AtoWCS       The world location of the geometry of object A.
    * @param A            The geometry of object A.
    * @param BtoWCS       The world location of the geometry of object B.
    * @param B            The geometry of object B.
    * @param contacts     Upon return holds all the contact points between the two object. By convention contact normals alway point from object A towards object B.
    * @param envelope     The size of the collision envelope. Whenever objects are within this distance then contact points will be generated.
    *
    * @return             If a collision is detected then the return value is true otherwise it is false.
    */
    template<typename coordsys_type,typename sdf_geometry_type,typename contact_point_container,typename real_type>
    bool sdf_sdf(
      coordsys_type const & AtoWCS
      , sdf_geometry_type const & A
      , coordsys_type const & BtoWCS
      , sdf_geometry_type const & B
      , contact_point_container & contacts
      , real_type const & envelope
      )
    {
      typedef typename sdf_geometry_type::bvh_type                               bvh_type;
      typedef          OpenTissue::sdf::CollisionPolicy<bvh_type,coordsys_type>  collision_policy;
      typedef          OpenTissue::bvh::SingleCollisionQuery< collision_policy > collision_query_type;

      coordsys_type AtoB;
      coordsys_type BtoA;
      AtoB = model_update(AtoWCS,BtoWCS);
      BtoA = inverse(AtoB);

      collision_query_type  query;
      contacts.clear();

      query.envelope()  = envelope;
      query.flipped()   = false;
      query.wcs_xform() = BtoWCS;
      query.run(AtoB, A.m_bvh, B, contacts);

      query.envelope()  = envelope;
      query.flipped()   = true; //--- We reversed the roles of objects A and B
      query.wcs_xform() = AtoWCS;
      query.run(BtoA, B.m_bvh, A, contacts);

      if(contacts.empty())
        return false;
      return true;
    }

  } //End of namespace collision
} // namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_SDF_SDF_H
#endif
