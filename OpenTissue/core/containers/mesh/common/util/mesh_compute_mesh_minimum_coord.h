#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MINIMUM_COORD_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MINIMUM_COORD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mesh
  {
    template<typename mesh_type, typename vector3_type>
    void compute_mesh_minimum_coord(mesh_type const & mesh, vector3_type & min_coord)
    {
      assert(mesh.size_vertices()>0 || !"mesh did not have any vertices");

      typename mesh_type::const_vertex_iterator end    = mesh.vertex_end();
      typename mesh_type::const_vertex_iterator v      = mesh.vertex_begin();
      min_coord = v->m_coord;
      for(;v!=end;++v)
        min_coord = min(min_coord, v->m_coord);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MINIMUM_COORD_H
#endif
