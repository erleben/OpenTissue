#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SHARING_VERTEX_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SHARING_VERTEX_H
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

    /**
    * Is Sharing A vertex.
    * This functions tests if the two specified faces is sharing
    * a common vertex on their boundaries.
    *
    * @param  f0   A constant reference to the first face.
    * @param  f1   A constant reference to the second face.
    *
    * @return      If the two faces is sharing a common vertex then
    *              the return value is true otherwise it is false,
    */
    template<typename mesh_type>
    bool is_sharing_vertex(PolyMeshFace<mesh_type> const & f0,PolyMeshFace<mesh_type> const & f1)
    {
      typedef typename mesh_type::const_face_vertex_circulator       const_face_vertex_circulator;
      typedef typename mesh_type::const_vertex_edge_circulator       const_vertex_edge_circulator;

      const_face_vertex_circulator v0(f0),end;
      for(;v0!=end;++v0)
        if( is_neighbor(f1,*v0) )
          return true;
      return false;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SHARING_VERTEX_H
#endif
