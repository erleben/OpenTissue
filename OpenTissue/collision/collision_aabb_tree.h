#ifndef OPENTISSUE_COLLISION_COLLISION_AABB_TREE_AGAINST_ITSELF_H
#define OPENTISSUE_COLLISION_COLLISION_AABB_TREE_AGAINST_ITSELF_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/aabb_tree/policies/aabb_tree_self_collision_policy.h>
#include <OpenTissue/collision/bvh/bvh_self_collision_query.h>

namespace OpenTissue
{
  namespace collision
  {

    /**
    * AABB Tree Collision Query.
    *
    * This method assumes that the AABB tree have been refitted prior to invokation.
    *
    * @param aabb_tree
    * @param contacts
    */
    template< typename aabb_tree_geometry, typename contact_point_container>
    void aabb_tree_against_itself( aabb_tree_geometry const & aabb_tree, contact_point_container & contacts)
    {
      typedef typename aabb_tree_geometry::bvh_type   bvh_type;

      typedef OpenTissue::bvh::SelfCollisionQuery< OpenTissue::aabb_tree::SelfCollisionPolicy<aabb_tree_geometry> >        collision_query;

      collision_query query;

      query.run(aabb_tree.m_bvh,contacts);
    }

  } // namespace collision

} // namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_AABB_TREE_AGAINST_ITSELF_H
#endif
