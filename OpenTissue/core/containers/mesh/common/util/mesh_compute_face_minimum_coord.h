#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_MINIMUM_COORD_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_MINIMUM_COORD_H
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
    template<typename face_type,typename vector3_type>
    void compute_face_minimum_coord(face_type const & f, vector3_type & min_coord)
    {
      typedef typename face_type::mesh_type               mesh_type;
      typedef typename mesh_type::face_vertex_circulator  face_vertex_circulator;
      face_vertex_circulator v(f),end;
      min_coord = v->m_coord;
      for(;v!=end;++v)
        min_coord = min(min_coord,v->m_coord);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_MINIMUM_COORD_H
#endif
