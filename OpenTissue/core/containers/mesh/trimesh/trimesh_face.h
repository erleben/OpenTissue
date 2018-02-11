#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_FACE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_FACE_H
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
    template<typename trimesh_type_ >
    class TriMeshFace : public trimesh_type_::face_traits
    {
    public:

      typedef trimesh_type_                          mesh_type;
      typedef typename mesh_type::face_handle        face_handle;
      typedef typename mesh_type::vertex_handle      vertex_handle;
      typedef typename mesh_type::vertex_iterator    vertex_iterator;

    private:

      face_handle        m_self;
      mesh_type       *  m_owner;
      vertex_handle      m_vertex0;
      vertex_handle      m_vertex1;
      vertex_handle      m_vertex2;

    public:
      TriMeshFace()
        : m_self()
        , m_owner(0)
        , m_vertex0()
        , m_vertex1()
        , m_vertex2()
      {};

    public:

      face_handle     get_handle() const { return m_self; };
      mesh_type    *  get_owner()  const { return m_owner; };

      vertex_handle   get_vertex0_handle()   const { return m_vertex0; };
      vertex_iterator get_vertex0_iterator() const { return m_owner->get_vertex_iterator(m_vertex0); };

      vertex_handle   get_vertex1_handle()   const { return m_vertex1; };
      vertex_iterator get_vertex1_iterator() const { return m_owner->get_vertex_iterator(m_vertex1); };

      vertex_handle   get_vertex2_handle()   const { return m_vertex2; };
      vertex_iterator get_vertex2_iterator() const { return m_owner->get_vertex_iterator(m_vertex2); };

    private:

      friend class trimesh_core_access;
      void set_handle(face_handle f) { m_self = f; };
      void set_owner(mesh_type * owner) { m_owner = owner; };
      void set_vertex0_handle(vertex_handle v) { m_vertex0 = v; };
      void set_vertex1_handle(vertex_handle v) { m_vertex1 = v; };
      void set_vertex2_handle(vertex_handle v) { m_vertex2 = v; };

    };

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_FACE_H
#endif
