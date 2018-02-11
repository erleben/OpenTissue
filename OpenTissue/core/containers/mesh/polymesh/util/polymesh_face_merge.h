#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_MERGE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_MERGE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_shared_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    * @param f0
    * @param f1
    *
    * @return
    */
    template<typename mesh_type>
    bool face_merge(PolyMeshFace<mesh_type> & f0,PolyMeshFace<mesh_type> & f1)
    {
      typedef typename mesh_type::face_halfedge_circulator        face_halfedge_circulator;
      typedef typename mesh_type::vertex_type                     vertex_type;
      typedef typename mesh_type::vertex_handle                   vertex_handle;
      typedef typename mesh_type::face_handle                     face_handle;
      typedef typename mesh_type::halfedge_type                   halfedge_type;

      mesh_type * owner = f0.get_owner();
      if(owner==0)
      {
        assert(!"face_merge(): No mesh owner!");
        return false;
      }

      if(!is_neighbor(f0,f1))
      {
        assert(!"face_merge(): faces were not neighbors");
        return false;
      }

      face_halfedge_circulator h(f0),hend;

      bool all_shared = true;
      for(;h!=hend;++h)
      {
        if(!is_shared_edge(*h,f0,f1))
        {
          all_shared = false;
          break;
        }
      }
      if(all_shared)
      {
        assert(!"face_merge(): faces shares all edges");
        return true;
      }

      while( !is_shared_edge(*h,f0,f1) )  ++h;

      while( is_shared_edge(*h,f0,f1)   )  --h;

      //--- now h points to the halfedge just before the first shared halfedge
      halfedge_type * before = &(*h);
      ++h;

      //--- h points to first shared edge, now collect all vertices
      //--- lying on shared boundary
      std::list<vertex_type * > shared_vertices;
      shared_vertices.push_back ( &(*(h->get_origin_iterator())) );
      while( is_shared_edge(*h,f0,f1)   )
      {
        shared_vertices.push_back ( &(*(h->get_destination_iterator())) );
        ++h;
      }

      //--- Now h points to the halfedge just after the last shared halfedge
      while( h->get_handle() != before->get_handle()   )
      {
        if( is_shared_edge(*h,f0,f1))
        {
          assert(!"face_merge(): illegal topology, multiple disjoint shared boundaries");
          return false;
        }
        ++h;
      }

      //--- At this point we know it is topological safe to merge the two faces

      //--- move two old faces
      owner->remove_face(f0.get_handle());
      owner->remove_face(f1.get_handle());

      //--- remove isolated vertices
      typename std::list<vertex_type * >::iterator  v   = shared_vertices.begin();
      typename std::list<vertex_type * >::iterator  end = shared_vertices.end();
      for(;v!=end;++v)
        if((*v)->get_outgoing_halfedge_handle().is_null())
          owner->remove_vertex((*v)->get_handle());

      std::list<vertex_handle> handles;
      halfedge_type * loop = before;
      do
      {
        handles.push_back(loop->get_destination_handle());
        loop = &(*(loop->get_next_iterator()));
      }while(loop!=before);

      face_handle merged_face = owner->add_face(handles.begin(),handles.end());
      if(merged_face.is_null())
      {
        assert(!"face_merge(): Could create merged face");
        return false;
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_MERGE_H
#endif
