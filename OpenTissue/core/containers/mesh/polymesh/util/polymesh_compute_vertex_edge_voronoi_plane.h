#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_EDGE_VORONOI_PLANE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_EDGE_VORONOI_PLANE_H
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
    template<typename vertex_type, typename halfedge_type, typename plane_type>
    void compute_vertex_edge_voronoi_plane(vertex_type const & v, halfedge_type const & h, plane_type & plane )
    {
      assert(v.get_handle() == h.get_destination_handle() || !"v must be destination of h");
      typename plane_type::vector3_type  u;
      compute_edge_direction(h,u);
      plane.set(u,v.m_coord);
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_EDGE_VORONOI_PLANE_H
#endif
