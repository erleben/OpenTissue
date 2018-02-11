#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MESH_TAUBIN_SMOOTH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MESH_TAUBIN_SMOOTH_H
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
    * @param N     Number of iterations.
    */
    template<typename mesh_type>
    void taubin_smooth(
      mesh_type & mesh
      , unsigned int N
      )
    {

      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename math_types::real_type                        real_type;

      typedef typename mesh_type::vertex_iterator                   vertex_iterator;
      typedef typename mesh_type::vertex_vertex_circulator          vertex_vertex_circulator;
      typedef typename mesh_type::index_type                        index_type;

      std::vector<vector3_type> laplacian(mesh.size_vertices());

      real_type zero = static_cast<real_type>(0.0);

      N*=2;
      for(unsigned int iteration=0;iteration<N;++iteration)
      {
        vertex_iterator vend = mesh.vertex_end();
        vertex_iterator v    = mesh.vertex_begin();
        for(int i=0;v!=vend;++v,++i)
        {
          laplacian[i].clear();

          if(is_boundary(*v))
            continue;
          vector3_type avg(zero,zero,zero);
          int n = 0;
          vertex_vertex_circulator  V(*v),Vend;
          for(;V!=Vend;++V,++n)
            avg += V->m_coord;
          avg *= (1.0/n);
          laplacian[i] = avg - v->m_coord;
        }
        if((iteration%2)==1)
        {
          vend = mesh.vertex_end();
          v    = mesh.vertex_begin();
          for(int i=0;v!=vend;++v,++i)
            v->m_coord -= .6731*laplacian[i];
        }
        else
        {
          vend = mesh.vertex_end();
          v    = mesh.vertex_begin();
          for(int i=0;v!=vend;++v,++i)
            v->m_coord += 0.6307*laplacian[i];
        }

      }
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MESH_TAUBIN_SMOOTH_H
#endif
