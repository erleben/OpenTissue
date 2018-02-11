#ifndef OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_REFIT_H
#define OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_REFIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/aabb_tree/policies/aabb_tree_refitter_policy.h>
#include <OpenTissue/collision/bvh/bvh_bottom_up_refitter.h>
#include <OpenTissue/collision/bvh/bvh_get_leaf_nodes.h>

namespace OpenTissue
{
  namespace aabb_tree
  {

    /**
    * Refit AABB Tree.
    *
    * @param aabb_tree
    */
    template<typename aabb_tree_geometry>
    void refit( aabb_tree_geometry  & aabb_tree)
    {
      typedef typename aabb_tree_geometry::bvh_type   bvh_type;

      typedef OpenTissue::bvh::BottomUpRefitter< OpenTissue::aabb_tree::RefitterPolicy<aabb_tree_geometry> >    refitter_type;

      // TODO exploit Caching, so leaves only have to be extracted once?
      typename bvh_type::bv_ptr_container leaves;
      OpenTissue::bvh::get_leaf_nodes(aabb_tree.m_bvh,leaves);
      refitter_type refitter;
      refitter.m_enlargement = 10e-5;
      refitter.run(leaves);
    }

  } // namespace aabb_tree

} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_REFIT_H
#endif
