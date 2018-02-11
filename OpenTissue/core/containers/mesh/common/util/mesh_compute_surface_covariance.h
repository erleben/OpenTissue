#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_SURFACE_COVARIANCE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_SURFACE_COVARIANCE_H
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

    /**
    * Covaraince of Surface.
    *
    * @param mesh
    * @param mu
    * @param C
    *
    */
    template<typename mesh_type, typename vector3_type, typename matrix3x3_type>
    void compute_surface_covariance(
      mesh_type const & mesh
      , vector3_type & mu
      , matrix3x3_type & cov
      )
    {
      typedef typename mesh_type::const_face_iterator            const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator   const_face_vertex_circulator;
      typedef typename vector3_type::value_type                  real_type;

      // Let n be number of triangles, a^i, b^i and c^i vertices of the i'th triangle
      // then the mean point is given
      //
      //  mu = frac{1}{3n} sum_i^n ( a^i + b^i + c^i)
      //
      // and the j,k entry of the convariance matrix is given by
      //
      //  C(j,k)  =  1/3n  sum_i^n ( A^i_j A^i_k + B^i_j B^i_k + C^i_j C^i_k )
      //
      // where
      //
      //   A = a - mu
      //   B = b - mu
      //   C = c - mu
      //
      mu.clear();
      unsigned int n=0;
      const_face_iterator end = mesh.face_end();
      const_face_iterator f   = mesh.face_begin();
      for(;f!=end;++f)
      {
        const_face_vertex_circulator  a(*f);--a;
        const_face_vertex_circulator  b(*f);
        const_face_vertex_circulator  c(*f);++c;
        for(;c->get_handle()!= a->get_handle(); ++b,++c )
        {
          mu += a->m_coord + b->m_coord + c->m_coord;
          ++n;
        }
      }
      real_type factor = static_cast<real_type>(1.0/(3.0*n));
      mu *= factor;
      cov.clear();
      for(f = mesh.face_begin();f!=end;++f)
      {
        const_face_vertex_circulator  a(*f);--a;
        const_face_vertex_circulator  b(*f);
        const_face_vertex_circulator  c(*f);++c;
        for(;c->get_handle()!= a->get_handle(); ++b,++c)
        {
          vector3_type A = a->m_coord - mu;
          vector3_type B = b->m_coord - mu;
          vector3_type C = c->m_coord - mu;
          for(unsigned int j=0;j<3;++j)
            for(unsigned int k=0;k<3;++k)
              cov(j,k) += A(j)*A(k) + B(j)*B(k) + C(j)*C(k);
        }
      }
      for(unsigned int j=0;j<3;++j)
        for(unsigned int k=0;k<3;++k)
          cov(j,k) *= factor;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_SURFACE_COVARIANCE_H
#endif
