#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_CORE_ACCESS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_CORE_ACCESS_H
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
  namespace trimesh
  {
    class trimesh_core_access
    {
    public:

      template<typename feature_iterator,typename handle>
      static void set_self_handle(feature_iterator feature,handle self) { feature->set_handle(self); }

      template<typename feature_iterator,typename mesh_type>
      static void set_owner(feature_iterator feature,mesh_type * owner){ feature->set_owner(owner); }

      template<typename face_iterator,typename vertex_handle>
      static void set_vertex0_handle(face_iterator f,vertex_handle v){ f->set_vertex0_handle(v); }

      template<typename face_iterator,typename vertex_handle>
      static void set_vertex1_handle(face_iterator f,vertex_handle v){ f->set_vertex1_handle(v); }

      template<typename face_iterator,typename vertex_handle>
      static void set_vertex2_handle(face_iterator f,vertex_handle v){ f->set_vertex2_handle(v); }

      template<typename vertex_iterator>
      static void increment_face_counter(vertex_iterator v){ ++(v->m_face_count); }

      template<typename vertex_iterator>
      static void decrement_face_counter(vertex_iterator v){ --(v->m_face_count); }
    };

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_CORE_ACCESS_H
#endif
