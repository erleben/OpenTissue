#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MEAN_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MEAN_H
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

    template<typename mesh_type, typename vector3_type>
    void compute_mesh_mean(mesh_type const & mesh, vector3_type & mean)
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::real_type                        real_type;

      assert(mesh.size_vertices()>0 || !"mesh did not have any vertices");

      typename mesh_type::const_vertex_iterator end    = mesh.vertex_end();
      typename mesh_type::const_vertex_iterator v      = mesh.vertex_begin();
      mean.clear();
      for(;v!=end;++v)
        mean +=  v->m_coord;

      mean /=  static_cast<real_type>(mesh.size_vertices());
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MEAN_H
#endif
