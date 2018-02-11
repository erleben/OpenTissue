#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_AABB_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_AABB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_mesh_minimum_coord.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_mesh_maximum_coord.h>

namespace OpenTissue
{
  namespace mesh
  {

    /**
     * Compute min max AABB Corners of Mesh.
     *
     * Example usage:
     *
     *  typedef ...   vector3_type;
     *  typedef ...   mesh_type;
     *
     *  mesh_type mesh;
     *  ...
     *
     *  vector3_type min_coord;
     *  vector3_type max_coord;
     *  OpenTissue::mesh::compute_aabb( mesh, min_coord , max_coord );
     *
     * @param mesh           A refence to a mesh.
     * @param min_coord      Upon return this argument holds the minimum coordinates of the mesh.
     * @param max_coord      Upon return this argument holds the maximum coordinates of the mesh.
     */
    template<typename mesh_type>
    inline void compute_aabb( 
        mesh_type const & mesh
        , typename mesh_type::math_types::vector3_type & min_coord
        , typename mesh_type::math_types::vector3_type & max_coord
      )
    {
      OpenTissue::mesh::compute_mesh_minimum_coord( mesh, min_coord );
      OpenTissue::mesh::compute_mesh_maximum_coord( mesh, max_coord );
    }

  } // namespace mesh
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_AABB_H
#endif
