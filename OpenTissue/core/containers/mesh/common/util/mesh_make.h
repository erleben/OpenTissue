#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_add2mesh.h>

namespace OpenTissue
{
  namespace mesh
  {

    /**
    * Mesh Make.
    * This function creates a new mesh from two other meshes.
    *
    * Example usage:
    *
    *  typedef OpenTissue::polymesh::PolyMesh<...> mesh_type;
    *  mesh_type A,B,C;
    *
    *  mesh::make(A,B,C);
    *
    * @param A     Reference to first input mesh.
    * @param B     Reference to other input mesh.
    * @param C     Reference to other input mesh.
    */
    template<typename mesh_type>
    inline void make(
        mesh_type const & A
      , mesh_type const & B
      , mesh_type & C
      )
    {
      C.clear();   
      add2mesh(A,C);
      add2mesh(B,C);
    }

    /**
    * Mesh Make.
    * This function creates a new mesh from two other meshes.
    *
    * Example usage:
    *
    *  typedef OpenTissue::polymesh::PolyMesh<...> mesh_type;
    *  mesh_type A,B,C;
    *
    *  C = mesh::make(A,B);
    *
    * @param A     Reference to first input mesh.
    * @param B     Reference to other input mesh.
    *
    * @return      The resulting mesh.
    */
    template<typename mesh_type>
    inline mesh_type make(
        mesh_type const & A
      , mesh_type const & B
      )
    {
      mesh_type C;
      make(A,B,C);
      return C;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_H
#endif
