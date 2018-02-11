#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_INTELLIGENT_PATCHER_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_INTELLIGENT_PATCHER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_clear_halfedge_tags.h>

#include <OpenTissue/core/geometry/geometry_plane.h> //--- needed for plane_type

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Plane Patcher Algorithm (Work in Progress).
    *
    *
    *
    */
    template< typename mesh_type >
    bool intelligent_patcher( mesh_type & mesh)
    {
      typedef typename mesh_type::halfedge_iterator  halfedge_iterator;
      typedef typename mesh_type::vertex_iterator    vertex_iterator;
      typedef typename mesh_type::halfedge_type      halfedge_type;
      typedef typename mesh_type::vertex_handle      vertex_handle;
      typedef typename mesh_type::index_type         index_type;

      typedef std::vector< vertex_handle >           ring_type;
      typedef std::vector< ring_type >               ring_container;
      typedef typename ring_container::iterator      ring_iterator;

      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::vector3_type              vector3_type;
      typedef typename math_types::real_type                 real_type;
      typedef geometry::Plane<math_types>                    plane_type;

      ring_container rings;

      //--- traverse open boundaries and collect ``rings'' of vertices.
      {
        mesh::clear_halfedge_tags(mesh);

        halfedge_iterator h = mesh.halfedge_begin();
        halfedge_iterator hend = mesh.halfedge_end();
        for(;h!=hend;++h)
        {
          index_type i = h->get_handle().get_idx();
          if(h->m_tag==1)
            continue;
          if(is_boundary(*h))
          {
            ring_type ring;
            halfedge_type * loop = &(*h);
            halfedge_type * first = loop;
            do
            {
              index_type j = loop->get_handle().get_idx();
              assert(loop->m_tag==0 || !"Oh we saw this halfedge before?");
              assert(is_boundary(*loop) || !"Oh, this edge must be a boundary, but it was not?");
              loop->m_tag = 1;
              ring.push_back(loop->get_destination_handle());
              loop = &(*(loop->get_next_iterator()));
            }while(loop!=first);
            if(ring.size()>2)
              rings.push_back(ring);
          }
          else
          {
            h->m_tag = 1;
          }
        }
      }


      //--- see if we have any more work or can quit now!
      unsigned int N = rings.size();
      if(N==0)
        return true;

      //--- for each ring, determine best fitting plane (normal)
      std::vector<plane_type>  planes;
      {
        ring_iterator r    = rings.begin();
        ring_iterator rend = rings.end();
        for(;r!=rend;++r)
        {
          ring_type & ring = (*r);
          plane_type plane;
          int i=0, j=1, n=r->size();
          real_type a = static_cast<real_type>(0.0);
          real_type b = static_cast<real_type>(0.0);
          real_type c = static_cast<real_type>(0.0);
          real_type xc = static_cast<real_type>(0.0);
          real_type yc = static_cast<real_type>(0.0);
          real_type zc = static_cast<real_type>(0.0);
          for(;i<n;++i,j=(j+1)%n)
          {
            vertex_iterator vi = mesh.get_vertex_iterator(  ring[i] );
            vertex_iterator vj = mesh.get_vertex_iterator(  ring[j] );
            real_type  xi = vi->m_coord(0);
            real_type  yi = vi->m_coord(1);
            real_type  zi = vi->m_coord(2);
            real_type  xj = vj->m_coord(0);
            real_type  yj = vj->m_coord(1);
            real_type  zj = vj->m_coord(2);
            a += (yi - yj) * (zi + zj);
            b += (zi - zj) * (xi + xj);
            c += (xi - xj) * (yi + yj);
            xc += xi;
            yc += yi;
            zc += zi;
          }
          real_type norm = static_cast<real_type>(std::sqrt(a*a+b*b+c*c));
          plane.n()(0) = a / norm;
          plane.n()(1) = b / norm;
          plane.n()(2) = c / norm;
          plane.w() = (plane.n()(0)*xc + plane.n()(0)*yc +plane.n()(0)*zc)/-n;
          planes.push_back(plane);
        }
      }

      //--- determine set of rings having same plane normal!!!
      std::vector< std::vector<ring_type *> >  planar_rings;
      {
        std::vector<bool> used(N);
        real_type tolerance = static_cast<real_type>(0.001);  //--- hmmm, is this a good tolerance
        for(int i=0;i<N;++i)
        {
          if(used[i])
            continue;
          planar_rings[i].push_back( rings[i] );
          used[i] = true;
          for(int j=i+1;j<N;++j)
          {
            real_type tst = planes[i].n()*planes[j].n();
            if(tst<tolerance)
            {
              planar_rings[i].push_back( rings[j] );
              used[j] = true;
            }
          }
        }
      }


      //--- for each set of ``planar'' rings build ring hierarchy


      //--- apply recursive divide and conquer algorithm to each ring hierarchy


      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_INTELLIGENT_PATCHER_H
#endif
