#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_NEIGHBOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_NEIGHBOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type>
    bool is_neighbor(PolyMeshFace<mesh_type> const & f, PolyMeshVertex<mesh_type> const & vertex)
    {
      typedef typename mesh_type::const_face_vertex_circulator       const_face_vertex_circulator;
      const_face_vertex_circulator v(f),end;
      for(; v!=end; ++v)
        if( v->get_handle().get_idx() == vertex.get_handle().get_idx())
          return true;
      return false;
    }

    template<typename mesh_type>
    bool is_neighbor(PolyMeshFace<mesh_type> const & f,PolyMeshHalfEdge<mesh_type> const & h)
    {
      if(h.get_face_handle().get_idx() == f.get_handle().get_idx())
        return true;
      return false;
    }

    template<typename mesh_type>
    bool is_neighbor(PolyMeshFace<mesh_type> const & face,PolyMeshEdge<mesh_type> const & e)
    {
      typedef typename mesh_type::halfedge_iterator halfedge_iterator;
      halfedge_iterator h0 = e.get_halfedge0_iterator();
      halfedge_iterator h1 = e.get_halfedge1_iterator();
      if(is_neighbor(face,*h0) || is_neighbor(face,*h1))
        return true;
      return false;
    }

    template<typename mesh_type>
    bool is_neighbor(PolyMeshFace<mesh_type> const & f0,PolyMeshFace<mesh_type> const & f1)
    {
      typedef typename mesh_type::const_face_edge_circulator       const_face_edge_circulator;
      const_face_edge_circulator e(f0),end;
      for(;e!=end;++e)
        if( is_neighbor(f1,*e))
          return true;
      return false;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_NEIGHBOR_H
#endif
