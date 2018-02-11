#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_VALENCY_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_VALENCY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/trimesh/trimesh_vertex.h>
#include <OpenTissue/core/containers/mesh/trimesh/trimesh_face.h>

namespace OpenTissue
{
  namespace trimesh
  {

    template<typename mesh_type>
    unsigned int valency(TriMeshVertex<mesh_type> const & v)
    {
      return v.get_face_count();
    }

    template<typename mesh_type>
    unsigned int valency(TriMeshFace<mesh_type> const & /*f*/)
    {
      return 3u;
    }

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_VALENCY_H
#endif
