#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_VERTEX_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_VERTEX_H
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

    template< typename polymesh_type_ >
    class PolyMeshVertex : public polymesh_type_::vertex_traits
    {
    public:

      typedef polymesh_type_                        mesh_type;
      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::halfedge_handle   halfedge_handle;
      typedef typename mesh_type::vertex_iterator   vertex_iterator;
      typedef typename mesh_type::halfedge_iterator halfedge_iterator;

    private:

      vertex_handle   m_self;
      mesh_type   *   m_owner;
      halfedge_handle m_outgoing_halfedge;

    public:

      PolyMeshVertex()
        : m_self()
        , m_owner(0)
        , m_outgoing_halfedge()
      {}

    public:

      vertex_handle     get_handle() const { return m_self; }
      mesh_type     *   get_owner() const { return m_owner; }
      halfedge_handle   get_outgoing_halfedge_handle() const { return m_outgoing_halfedge; }
      halfedge_iterator get_outgoing_halfedge_iterator() const { return m_owner->get_halfedge_iterator(m_outgoing_halfedge); }

    private:

      friend class polymesh_core_access;
      void set_handle(vertex_handle v) { m_self = v; }
      void set_owner(mesh_type * owner) { m_owner = owner; }
      void set_outgoing_halfedge_handle(halfedge_handle h) { m_outgoing_halfedge = h; }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_VERTEX_H
#endif
