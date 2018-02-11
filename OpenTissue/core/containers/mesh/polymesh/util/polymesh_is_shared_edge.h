#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SHARED_EDGE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SHARED_EDGE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_neighbor.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type>
    bool is_shared_edge(PolyMeshEdge<mesh_type> const & e,PolyMeshFace<mesh_type> const & f0,PolyMeshFace<mesh_type> const & f1)
    {
      if(is_neighbor(f0,e) && is_neighbor(f1,e))
        return true;
      return false;
    }

    template<typename mesh_type>
    bool is_shared_edge(PolyMeshHalfEdge<mesh_type> const & h,PolyMeshFace<mesh_type> const & f0,PolyMeshFace<mesh_type> const & f1)
    {
      typedef typename mesh_type::edge_iterator edge_iterator;
      edge_iterator e = h.get_edge_iterator();
      return is_shared_edge(*e,f0,f1);
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SHARED_EDGE_H
#endif
