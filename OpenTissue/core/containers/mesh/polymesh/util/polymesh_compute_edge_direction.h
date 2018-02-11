#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_EDGE_DIRECTION_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_EDGE_DIRECTION_H
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
  namespace polymesh
  {

    template<typename halfedge_type, typename vector3_type>
    void compute_edge_direction(halfedge_type const & h,vector3_type & u)
    {
      u = h.get_destination_iterator()->m_coord - h.get_origin_iterator()->m_coord;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_EDGE_DIRECTION_H
#endif
