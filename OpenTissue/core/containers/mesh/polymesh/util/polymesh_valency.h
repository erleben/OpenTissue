#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_VALENCY_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_VALENCY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type>
    unsigned int valency(PolyMeshVertex<mesh_type> const & v)
    {
      typedef typename mesh_type::vertex_halfedge_circulator   vertex_halfedge_circulator;
      unsigned int valency = 0;
      vertex_halfedge_circulator h(v), end;
      for(;h!=end;++h,++valency);
      return valency;
    }

    template<typename mesh_type>
    unsigned int valency(PolyMeshFace<mesh_type> const & f)
    {
      typedef typename mesh_type::face_halfedge_circulator   face_halfedge_circulator;
      unsigned int valency = 0;
      face_halfedge_circulator h(f),end;
      for(;h!=end;++h,++valency);
      return valency;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_VALENCY_H
#endif
