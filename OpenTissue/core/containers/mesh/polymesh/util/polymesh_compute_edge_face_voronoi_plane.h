#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_EDGE_FACE_VORONOI_PLANE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_EDGE_FACE_VORONOI_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_direction.h>

namespace OpenTissue
{
  namespace polymesh
  {
    template<typename halfedge_type, typename face_type, typename plane_type>
    void compute_edge_face_voronoi_plane( halfedge_type const & h,face_type const & f, plane_type & plane)
    {
      assert(h.get_face_handle() == f.get_handle() || !"h must be border of f");
      typename plane_type::vector3_type n,u;

      compute_face_normal(f,n);
      compute_edge_direction(h,u);
      typename plane_type::vector3_type m = cross(n , u);
      plane.set(m,h.get_origin_iterator()->m_coord);
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_EDGE_FACE_VORONOI_PLANE_H
#endif
