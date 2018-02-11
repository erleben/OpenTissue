#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_NAIVE_PATCHER_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_NAIVE_PATCHER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_plane.h> //--- needed for plane_type

namespace OpenTissue
{
  namespace polymesh
  {

    template< typename mesh_type >
    bool naive_patcher( mesh_type  & mesh)
    {
      typedef typename mesh_type::halfedge_iterator halfedge_iterator;
      typedef typename mesh_type::halfedge_type     halfedge_type;
      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::index_type        index_type;

      typedef std::list< vertex_handle >  ring_type;
      typedef std::list< ring_type >      ring_container;
      typedef typename ring_container::iterator    ring_iterator;

      ring_container      rings;

      //--- traverse open boundaries and collect ``rings'' of vertices.
      mesh::clear_halfedge_tags(mesh);

      halfedge_iterator h = mesh.halfedge_begin();
      halfedge_iterator hend = mesh.halfedge_end();
      for(;h!=hend;++h)
      {
        if(h->m_tag == 1)
          continue;

        if(is_boundary(*h))
        {
          rings.push_back(ring_type());
          ring_type & ring = rings.back();
          halfedge_type * loop = &(*h);
          halfedge_type * first = loop;
          do
          {
            assert(loop->m_tag==1 || !"Oh we saw this halfedge before?");
            assert(is_boundary(*loop) || !"Oh, this edge must be a boundary, but it was not?");
            loop->m_tag=1;
            ring.push_back(loop->get_destination_handle());
            loop = &(*(loop->get_next_iterator()));
          }while(loop!=first);
        }
        else
        {
          h->m_tag=1;
        }
      }

      //--- for each ring, determine best fitting plane
      std::size_t N = rings.size();
      if(N==0)
        return true;

      ring_iterator r = rings.begin();
      ring_iterator rend = rings.end();

      for(;r!=rend;++r)
      {
        if(r->size()>2)
        {
          mesh.add_face(r->begin(),r->end());
        }
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_NAIVE_PATCHER_H
#endif
