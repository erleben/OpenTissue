#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_PLANE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_face_normal.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_center.h>

namespace OpenTissue
{
  namespace mesh
  {

    template<typename face_type,typename plane_type>
    void compute_face_plane(face_type const & f, plane_type & plane)
    {
      typedef typename face_type::mesh_type                mesh_type;
      typedef typename mesh_type::math_types               math_types;
      typedef typename math_types::value_traits            value_traits;
      typedef typename math_types::vector3_type            vector3_type;
      typedef typename math_types::real_type               real_type;

      vector3_type n,c;
      compute_face_center(f,c);
      compute_face_normal(f,n);
      plane.set(n,c);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_PLANE_H
#endif
