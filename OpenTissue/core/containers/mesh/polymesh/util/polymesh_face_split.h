#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SPLIT_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SPLIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    * @param f
    * @param v0
    * @param v1
    *
    * @return
    */
    template<typename mesh_type>
    bool face_split(PolyMeshFace<mesh_type> & f,PolyMeshVertex<mesh_type> & v0,PolyMeshVertex<mesh_type> & v1)
    {
      typedef typename mesh_type::face_halfedge_circulator        face_halfedge_circulator;
      typedef typename mesh_type::halfedge_type                   halfedge_type;
      typedef typename mesh_type::vertex_handle                   vertex_handle;
      typedef typename mesh_type::face_handle                     face_handle;

      mesh_type * owner = f.get_owner();
      if(owner==0)
      {
        assert(!"face_split(): No mesh owner!");
        return false;
      }
      if(valency(f)==3)
      {
        assert(!"face_split(): face was a triangle");
        return false;
      }
      if(v0.get_handle() == v1.get_handle())
      {
        assert(!"face_split(): vertices were identical");
        return false;
      }

      halfedge_type * h0 = 0;
      halfedge_type * h1 = 0;
      face_halfedge_circulator h(f),hend;

      for(;h!=hend;++h)
      {
        if(h->get_origin_handle() == v0.get_handle())
          h0 = &(*h);
        if(h->get_origin_handle() == v1.get_handle())
          h1 = &(*h);
      }
      if( ( !h0 || !h1 ) || h1==h0 )
      {
        assert(!"face_split(): could not detect new face boundaries");
        return false;
      }

      halfedge_type * h0_end = &(*(h1->get_prev_iterator()));
      halfedge_type * h1_end = &(*(h0->get_prev_iterator()));

      //--- Traverse boundary of new faces
      std::list<vertex_handle> handles0;
      std::list<vertex_handle> handles1;

      halfedge_type * loop = h0;
      handles0.push_back(h0->get_origin_handle());
      while(loop!=h0_end)
      {
        handles0.push_back(loop->get_destination_handle());
        loop = &(*(loop->get_next_iterator()));
      }
      handles0.push_back(h0_end->get_destination_handle());


      if(handles0.size()<3)
      {
        assert(!"face_split(): split would result in degenerate face");
        return false;
      }

      loop = h1;
      handles1.push_back(h1->get_origin_handle());
      while(loop!=h1_end)
      {
        handles1.push_back(loop->get_destination_handle());
        loop = &(*(loop->get_next_iterator()));
      }
      handles1.push_back(h1_end->get_destination_handle());

      if(handles1.size()<3)
      {
        assert(!"face_split(): split would result in degenerate face");
        return false;
      }

      //--- move two old face
      owner->remove_face(f.get_handle());

      face_handle f0 = owner->add_face(handles0.begin(),handles0.end());
      if(f0.is_null())
      {
        assert(!"face_split(): Could create face 0");
        return false;
      }

      face_handle f1 = owner->add_face(handles1.begin(),handles1.end());
      if(f1.is_null())
      {
        assert(!"face_split(): Could create face 1");
        return false;
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SPLIT_H
#endif
