#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_COMPUTE_FACE_NORMAL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_COMPUTE_FACE_NORMAL_H
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
    template<typename mesh_type,typename vector3_type>
    void compute_face_normal(TriMeshFace<mesh_type> const & f, vector3_type & normal)
    {
      typedef typename mesh_type::vertex_iterator   vertex_iterator;
      vertex_iterator v0 = f.get_vertex0_iterator();
      vertex_iterator v1 = f.get_vertex1_iterator();
      vertex_iterator v2 = f.get_vertex2_iterator();
      vector3_type u1  =  v1->m_coord - v0->m_coord;
      vector3_type u2  =  v2->m_coord - v1->m_coord;
      normal           =  unit( cross(u1, u2) );
    }

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_COMPUTE_FACE_NORMAL_H
#endif
