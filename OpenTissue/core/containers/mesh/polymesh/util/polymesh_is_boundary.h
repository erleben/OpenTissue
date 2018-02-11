#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_BOUNDARY_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_BOUNDARY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type>
    bool is_boundary(PolyMeshHalfEdge<mesh_type> const & h)
    {
      if(h.get_face_handle().is_null())
        return true;
      return false;
    }

    template<typename mesh_type>
    bool is_boundary(PolyMeshVertex<mesh_type> const & v)
    {
      typedef typename mesh_type::vertex_halfedge_circulator   vertex_halfedge_circulator;

      vertex_halfedge_circulator h(v), end;
      bool got_edges = false;
      for(;h!=end;++h)
      {
        got_edges = true;
        if( is_boundary( *h ) )
          return true;
      }
      if(got_edges)
        return false;
      return true;
    }

    template<typename mesh_type>
    bool is_boundary(PolyMeshEdge<mesh_type> const & e)
    {
      typedef typename mesh_type::halfedge_iterator   halfedge_iterator;

      halfedge_iterator h0 = e.get_halfedge0_iterator();
      if( is_boundary( *h0 ) )
        return true;
      halfedge_iterator h1 = e.get_halfedge1_iterator();
      if( is_boundary( *h1 ) )
        return true;
      return false;
    }

    template<typename mesh_type>
    bool is_boundary(PolyMeshFace<mesh_type> const & f)
    {
      typedef typename mesh_type::face_halfedge_circulator   face_halfedge_circulator;

      face_halfedge_circulator h(f),end;
      for( ; h!=end; ++h)
      {
        if(    is_boundary(     *(h->get_twin_iterator())      )    )
          return true;
      }
      return false;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_BOUNDARY_H
#endif
