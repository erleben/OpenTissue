#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MESH_LAPLACIAN_SMOOTH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MESH_LAPLACIAN_SMOOTH_H
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

    /**
    * Requires that mesh supports: is_boundary(vertex_type) and vertex_vertex_circulator
    *
    * @param mesh
    * @param t
    */
    template<typename mesh_type>
    void laplacian_smooth(
      mesh_type & mesh
      , double  t
      )
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename math_types::real_type                        real_type;
      typedef typename mesh_type::vertex_iterator                   vertex_iterator;
      typedef typename mesh_type::vertex_vertex_circulator          vertex_vertex_circulator;

      t = std::min(t, 1.0);
      double ti = 1.0 - t;

      real_type zero = static_cast<real_type>(0.0);

      vertex_iterator vend = mesh.vertex_end();
      vertex_iterator v    = mesh.vertex_begin();
      for(;v!=vend;++v)
      {
        if(is_boundary(*v))
          continue;
        int n = 0;
        vector3_type avg(zero,zero,zero);
        vertex_vertex_circulator  V(*v),Vend;
        for(;V!=Vend;++V,++n)
        {
          avg += V->m_coord;
        }
        v->m_coord = t * avg/n + ti * v->m_coord;
      }
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MESH_LAPLACIAN_SMOOTH_H
#endif
