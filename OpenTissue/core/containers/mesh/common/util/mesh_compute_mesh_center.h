#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_CENTER_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_CENTER_H
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

    //    template<typename mesh_type, typename vector3_type>
    template<typename mesh_type>
      void compute_mesh_center(mesh_type const & mesh 
                               , typename mesh_type::math_types::vector3_type & center)
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename math_types::real_type                        real_type;

      vector3_type min_coord;
      vector3_type max_coord;
      mesh::compute_mesh_minimum_coord(mesh,min_coord);
      mesh::compute_mesh_maximum_coord(mesh,max_coord);
      center = (max_coord+min_coord)/value_traits::two();
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_CENTER_H
#endif
