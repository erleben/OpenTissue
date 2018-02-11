#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_EDGE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_EDGE_H
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
    class PolyMeshEdge : public polymesh_type_::edge_traits
    {
    public:

      typedef polymesh_type_                 mesh_type;
      typedef typename mesh_type::halfedge_handle     halfedge_handle;
      typedef typename mesh_type::halfedge_iterator   halfedge_iterator;
      typedef typename mesh_type::edge_handle         edge_handle;
      typedef typename mesh_type::edge_iterator       edge_iterator;

    private:

      edge_handle      m_self;
      mesh_type     *  m_owner;
      halfedge_handle  m_halfedge0;
      halfedge_handle  m_halfedge1;

    public:

      PolyMeshEdge() 
        : m_self()
        , m_owner(0)
        , m_halfedge0()
        , m_halfedge1()
      {}

    public:

      edge_handle       get_handle() const { return m_self; }
      mesh_type     *   get_owner() const { return m_owner; }

      halfedge_handle   get_halfedge0_handle() const { return m_halfedge0; }
      halfedge_iterator get_halfedge0_iterator() const { return m_owner->get_halfedge_iterator(m_halfedge0); }

      halfedge_handle   get_halfedge1_handle() const { return m_halfedge1; };
      halfedge_iterator get_halfedge1_iterator() const { return m_owner->get_halfedge_iterator(m_halfedge1); }


    private:

      friend class polymesh_core_access;
      void set_handle(edge_handle e) { m_self = e; }
      void set_owner(mesh_type * owner) { m_owner = owner; }
      void set_halfedge0_handle(halfedge_handle h) { m_halfedge0 = h; }
      void set_halfedge1_handle(halfedge_handle h) { m_halfedge1 = h; }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_EDGE_H
#endif
