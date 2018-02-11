#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_GET_FACE_VERTICES_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_GET_FACE_VERTICES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type, typename vertex_ptr_container>
    void get_face_vertices(PolyMeshFace<mesh_type> & face, vertex_ptr_container & vertices)
    {
      typedef typename mesh_type::face_vertex_circulator       face_vertex_circulator;
      vertices.clear();    
      face_vertex_circulator v(face),end();
      for(;v!=end;++v)
        vertices.push_back( &(*v) );
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_GET_FACE_VERTICES_H
#endif
