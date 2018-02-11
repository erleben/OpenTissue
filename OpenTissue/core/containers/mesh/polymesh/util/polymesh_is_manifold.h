#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_MANIFOLD_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_MANIFOLD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template< typename mesh_type  >
    bool is_manifold( mesh_type const & mesh )
    {
      typename mesh_type::const_vertex_iterator vend   = mesh.vertex_end();
      typename mesh_type::const_vertex_iterator v     = mesh.vertex_begin();
      for(;v!=vend;++v)
      {
        if(is_boundary( *v ) )
          return false;
      }
      typename mesh_type::const_halfedge_iterator hend   = mesh.halfedge_end();
      typename mesh_type::const_halfedge_iterator h     = mesh.halfedge_begin();
      for(;h!=hend;++h)
      {
        if(is_boundary( *h ) )
          return false;
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_MANIFOLD_H
#endif
