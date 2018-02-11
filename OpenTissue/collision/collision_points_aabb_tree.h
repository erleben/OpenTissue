#ifndef OPENTISSUE_COLLISION_COLLISION_POINTS_AABB_TREE_H
#define OPENTISSUE_COLLISION_COLLISION_POINTS_AABB_TREE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/aabb_tree/policies/aabb_tree_single_collision_policy.h>
#include <OpenTissue/collision/bvh/bvh_single_collision_query.h>

namespace OpenTissue
{
  namespace collision
  {


    /**
    * AABB Tree Collision Query.
    * This method assumes that the two AABB trees have been refitted prior to invokation.
    *
    * @param begin
    * @param end
    * @param aabb_tree
    * @param contacts
    */
    template<typename point_iterator, typename aabb_tree_geometry,typename contact_point_container>
    void points_aabb_tree( 
      point_iterator begin
      , point_iterator end
      , aabb_tree_geometry const & aabb_tree
      , contact_point_container & contacts
      )
    {
      typedef typename aabb_tree_geometry::bvh_type   bvh_type;

      typedef OpenTissue::bvh::SingleCollisionQuery< OpenTissue::aabb_tree::SingleCollisionPolicy<aabb_tree_geometry> >      collision_query;

      int dont_care_identity_transform;

      collision_query query;

      for(point_iterator p = begin;p!=end;++p)
        query.run( dont_care_identity_transform, aabb_tree.m_bvh, (*p), contacts);

    }

  } // namespace collision

} // namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_POINTS_AABB_TREE_H
#endif
