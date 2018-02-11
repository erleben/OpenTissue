#ifndef OPENTISSUE_COLLISION_SDF_SDF_DEBUG_DRAW_BVH_H
#define OPENTISSUE_COLLISION_SDF_SDF_DEBUG_DRAW_BVH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/gl/gl_util.h>
#include <OpenTissue/collision/bvh/bvh_get_nodes_at_depth.h>
#include <OpenTissue/collision/bvh/bvh_get_nodes_at_height.h>

namespace OpenTissue
{
  namespace sdf
  {


    /**
    * BVH Debug Drawing Utility.
    * This function can be used to draw sphere geometries at a specified
    * depth level in the bounding volume hiearchy (BVH) of a signed distance
    * field geometry.
    *
    * @param geometry     The signed distance field geometry.
    * @param depth        The depth level of the BVH that should be drawn.
    */
    template<typename sdf_geometry_type>
    void debug_draw_bvh(sdf_geometry_type const & geometry, int depth)
    {
      typedef typename sdf_geometry_type::bvh_type    bvh_type;
      typedef typename bvh_type::bv_ptr_container     bv_ptr_container;
      typedef typename bvh_type::bv_iterator          bv_iterator;

      bv_ptr_container nodes;

      bvh::get_nodes_at_height(geometry.m_bvh,1,nodes);
      //gl::ColorPicker(0.1,0.8,0.1);
      {
        bv_iterator node = nodes.begin();
        bv_iterator end = nodes.end();
        for (;node!= end;++node )
          gl::DrawSphere( node->volume(), false );
      }
      bvh::get_nodes_at_depth(geometry.m_bvh,depth,nodes);
      //gl::ColorPicker(0.8,0.4,0.1);
      {
        bv_iterator node = nodes.begin();
        bv_iterator end = nodes.end();
        for (;node!= end;++node )
          gl::DrawSphere( node->volume(), true );
      }
    }

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_DEBUG_DRAW_BVH_H
#endif
