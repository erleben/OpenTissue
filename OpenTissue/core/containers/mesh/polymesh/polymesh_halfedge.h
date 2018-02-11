#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_HAFLEDGE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_HAFLEDGE_H
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

    template<typename polymesh_type_ >
    class PolyMeshHalfEdge : public polymesh_type_::halfedge_traits
    {
    public:

      typedef polymesh_type_               mesh_type;

      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::halfedge_handle   halfedge_handle;
      typedef typename mesh_type::edge_handle       edge_handle;
      typedef typename mesh_type::face_handle       face_handle;

      typedef typename mesh_type::vertex_iterator   vertex_iterator;
      typedef typename mesh_type::halfedge_iterator halfedge_iterator;
      typedef typename mesh_type::edge_iterator     edge_iterator;
      typedef typename mesh_type::face_iterator     face_iterator;

    private:

      halfedge_handle m_self;
      mesh_type     * m_owner;
      halfedge_handle m_next;
      halfedge_handle m_prev;
      face_handle     m_face;
      halfedge_handle m_twin;
      vertex_handle   m_destination;
      edge_handle     m_edge;

    public:

      PolyMeshHalfEdge() 
        : m_self()
        , m_owner(0)
        , m_next()
        , m_prev()
        , m_face()
        , m_twin()
        , m_destination()
        , m_edge()
      {}

    public:

      halfedge_handle   get_handle() const { return m_self; }
      mesh_type     *   get_owner() const { return m_owner; }

      halfedge_handle   get_next_handle() const { return m_next; }
      halfedge_iterator get_next_iterator() const { return m_owner->get_halfedge_iterator(m_next); }

      halfedge_handle   get_twin_handle() const { return m_twin; }
      halfedge_iterator get_twin_iterator() const { return m_owner->get_halfedge_iterator(m_twin); }

      vertex_handle     get_origin_handle() const { return get_twin_iterator()->get_destination_handle(); }
      vertex_iterator   get_origin_iterator() const { return get_twin_iterator()->get_destination_iterator(); }

      vertex_handle     get_destination_handle() const { return m_destination; }
      vertex_iterator   get_destination_iterator() const { return m_owner->get_vertex_iterator(m_destination); }

      face_handle       get_face_handle() const { return m_face; }
      face_iterator     get_face_iterator() const { return m_owner->get_face_iterator(m_face); }

      halfedge_handle   get_prev_handle() const { return m_prev; }
      halfedge_iterator get_prev_iterator() const { return m_owner->get_halfedge_iterator(m_prev); }

      edge_handle       get_edge_handle() const { return m_edge; }
      edge_iterator     get_edge_iterator() const { return m_owner->get_edge_iterator(m_edge); }

    private:

      friend class polymesh_core_access;
      void set_handle(halfedge_handle h) { m_self = h; }
      void set_owner(mesh_type * owner){ m_owner = owner; }
      void set_next_handle(halfedge_handle h)
      {
        //if(m_owner->is_valid_halfedge_handle(m_next))
        //{
        //   m_owner->get_halfedge_iterator(m_next)->m_prev = m_owner->null_halfedge_handle;
        //}

        m_next = h;

        if(m_owner->is_valid_halfedge_handle(h))
        {
          m_owner->get_halfedge_iterator(h)->m_prev = m_self;
        }      
      }
      void set_face_handle(face_handle f){ m_face = f; }
      void set_twin_handle(halfedge_handle h){ m_twin = h; }
      void set_destination_handle(vertex_handle v){ m_destination = v; }
      void set_edge_handle(edge_handle e){ m_edge = e; }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_HAFLEDGE_H
#endif
