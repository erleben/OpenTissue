#ifndef OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_INIT_H
#define OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_INIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph.h>
#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_bottom_up_constructor.h>
#include <OpenTissue/collision/aabb_tree/policies/aabb_tree_bottom_up_constructor_policy.h>
#include <OpenTissue/collision/aabb_tree/policies/aabb_tree_graph_converter.h>

namespace OpenTissue
{
  namespace aabb_tree
  {

    /**
    * Initialize AABB Tree.
    *
    * @param mesh
    * @param aabb_tree
    */
    template< typename mesh_type, typename aabb_tree_geometry, typename vertex_data_binder>
    void init(mesh_type & mesh,aabb_tree_geometry & aabb_tree, vertex_data_binder & binder)
    {

      typedef typename aabb_tree_geometry::bvh_type                                                                          bvh_type;
      typedef OpenTissue::bvh::BVHGraph<bvh_type>                                                                            graph_type;
      typedef OpenTissue::bvh::BottomUpConstructor<bvh_type, OpenTissue::aabb_tree::BottomUpConstructorPolicy<bvh_type> >    constructor_type;
      typedef OpenTissue::aabb_tree::GraphConverter<graph_type>                                                              converter_type;

      graph_type       graph;               ///< Graph data structure used as initial input for the bottom-up constructor.
      converter_type   converter;           ///< A volume to graph conversion utility. Takes a set of initial volumes and creates a graph.
      constructor_type constructor;         ///< The bottom up constructor.

      converter.run( mesh, graph, binder );
      constructor.run( graph, aabb_tree.m_bvh );
    }


  } // namespace aabb_tree
} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_INIT_H
#endif
