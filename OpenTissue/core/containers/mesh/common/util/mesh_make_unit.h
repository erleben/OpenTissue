#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_UNIT_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_UNIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_mesh_center.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_mesh_minimum_coord.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_mesh_maximum_coord.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_deformation_modifiers.h>

#include <cassert>
#include <cmath>

namespace OpenTissue
{
  namespace mesh
  {

    /**
    * Center mesh around origin and rescale mesh so it is contained inside an unit AABB centered at the origin.
    *
    * Note that the rescaling preserves the aspect ratio of the mesh.
    *
    */
    template<typename mesh_type>
    void make_unit(mesh_type & mesh)
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;


      vector3_type min_coord;
      vector3_type max_coord;
      vector3_type center;
      vector3_type range;
      vector3_type s(1.0,1.0,1.0);

      mesh::compute_mesh_center(mesh,center);
      mesh::compute_mesh_minimum_coord(mesh,min_coord);
      mesh::compute_mesh_maximum_coord(mesh,max_coord);

      range = max_coord - min_coord;
      typename vector3_type::value_type max_range = std::max(range(0),std::max(range(1),range(2)));
      if(range(0))
        s(0) = 1.0/max_range;
      if(range(1))
        s(1) = 1.0/max_range;
      if(range(2))
        s(2) = 1.0/max_range;
      mesh::translate(mesh,-center);
      mesh::scale(mesh, s);
      //translate(mesh,center);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_UNIT_H
#endif
