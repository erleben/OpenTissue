#ifndef OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_DEBUG_DRAW_H
#define OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_DEBUG_DRAW_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_get_leaf_nodes.h>

namespace OpenTissue
{
  namespace aabb_tree
  {

    /**
    * Debug Draw.
    *
    * @param aabb_tree
    */
    template<typename aabb_tree_geometry>
    void debug_draw( aabb_tree_geometry const & aabb_tree)
    {
      typedef typename aabb_tree_geometry::bvh_type   bvh_type;

      typename bvh_type::bv_ptr_container nodes;
      OpenTissue::bvh::get_leaf_nodes(aabb_tree.m_bvh,nodes);

      typename bvh_type::bv_iterator node = nodes.begin();
      typename bvh_type::bv_iterator end  = nodes.end();
      for (;node!= end;++node )
        node->volume().draw( GL_LINE_LOOP );
    }

  } // namespace aabb_tree
} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_DEBUG_DRAW_H
#endif
