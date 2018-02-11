#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_FACE_FLIP_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_FACE_FLIP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/trimesh/trimesh_face.h>

namespace OpenTissue
{
  namespace trimesh
  {

    template<typename mesh_type>
    bool face_flip(TriMeshFace<mesh_type> const & f)
    {
      typedef typename TriMeshFace<mesh_type>::vertex_handle vertex_handle;
      vertex_handle h0 = f.get_vertex0_handle();
      vertex_handle h1 = f.get_vertex1_handle();
      //vertex_handle h2 = f.get_vertex2_handle();
      trimesh_core_access::set_vertex0_handle( &f, h1 );
      trimesh_core_access::set_vertex1_handle( &f, h0 );
      //trimesh_core_access::set_vertex2_handle( &f, h2 );
      return true;
    }

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_FACE_FLIP_H
#endif
