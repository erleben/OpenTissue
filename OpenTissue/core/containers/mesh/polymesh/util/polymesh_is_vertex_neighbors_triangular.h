#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_VERTEX_NEIGHBORS_TRIANGULAR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_VERTEX_NEIGHBORS_TRIANGULAR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Triangular Neighborhood Test Function.
    * Test if all neighboring faces of a vertex are triangles.
    *
    * @param v      The specified vertex.
    *
    * @return       If all faces are triangles then the return value is true otherwise it is false.
    */
    template<typename mesh_type>
    bool is_vertex_neighbors_triangular(PolyMeshVertex<mesh_type> const & v)
    {
      typedef typename mesh_type::vertex_face_circulator       vertex_face_circulator;
      vertex_face_circulator f(v),fend;
      for(;f!=fend;++f)
        if(valency(*f)!=3)
          return false;
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_VERTEX_NEIGHBORS_TRIANGULAR_H
#endif
