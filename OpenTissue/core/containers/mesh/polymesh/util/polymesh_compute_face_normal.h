#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_FACE_NORMAL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_FACE_NORMAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <stdexcept>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type,typename vector3_type>
    void compute_face_normal(PolyMeshFace<mesh_type> const & f, vector3_type & normal)
    {
      typedef typename mesh_type::face_vertex_circulator   face_vertex_circulator;
      typedef typename vector3_type::value_type            real_type;


      // From Mantyla 88:
      //
      //HalfEdge        *he;
      //double          a, b, c, norm;
      //double          xi, yi, zi, xj, yj, zj, xc, yc, zc;
      //int             length;
      //
      //a = b = c = xc = yc = zc = 0.0;
      //length = 0;
      //he = l->ledg;
      //do
      //{
      //        xi = he->vtx->vcoord[0];
      //        yi = he->vtx->vcoord[1];
      //        zi = he->vtx->vcoord[2];
      //        xj = he->nxt->vtx->vcoord[0];
      //        yj = he->nxt->vtx->vcoord[1];
      //        zj = he->nxt->vtx->vcoord[2];
      //        a += (yi - yj) * (zi + zj);
      //        b += (zi - zj) * (xi + xj);
      //        c += (xi - xj) * (yi + yj);
      //        xc += xi;
      //        yc += yi;
      //        zc += zi;
      //        length++;
      //}
      //while((he = he->nxt) != l->ledg);
      //
      //if((norm = sqrt(a*a + b*b + c*c)) != 0.0)
      //{
      //        eq[0] = a / norm;
      //        eq[1] = b / norm;
      //        eq[2] = c / norm;
      //        eq[3] = (eq[0]*xc + eq[1]*yc + eq[2]*zc) / (-length);
      //return(SUCCESS);
      //}
      //else
      //{
      //        printf("faceeq: null face %d\n", l->lface->faceno);
      //        return(ERROR);
      //}




      normal.clear();

      if(valency(f)<=2)
      {
        throw std::invalid_argument( "compute_face_plane(): Face has less than three vertices!" );
        return;
      }

      face_vertex_circulator cur(f),end;
      face_vertex_circulator prev(f);--prev;
      face_vertex_circulator next(f);++next;

      real_type max_area = static_cast<real_type>(0.0);

      for(;cur!=end; ++cur,++next,++prev)
      {
        vector3_type u1       =  cur->m_coord - prev->m_coord;
        vector3_type u2       =  next->m_coord - cur->m_coord;
        vector3_type u1xu2    = u1 % u2;
        real_type    area_sqr = u1xu2*u1xu2;
        if(area_sqr > max_area)
        {
          max_area = area_sqr;
          normal = normalize(u1xu2);
        }
      }
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_FACE_NORMAL_H
#endif
