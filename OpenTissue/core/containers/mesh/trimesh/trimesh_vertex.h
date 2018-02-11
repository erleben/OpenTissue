#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_VERTEX_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_VERTEX_H
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
    template< typename trimesh_type_ >
    class TriMeshVertex : public trimesh_type_::vertex_traits
    {
    public:

      typedef trimesh_type_                       mesh_type;
      typedef typename mesh_type::vertex_handle   vertex_handle;

    private:

      vertex_handle    m_self;
      mesh_type    *   m_owner;
      unsigned int     m_face_count;

    public:

      TriMeshVertex()
        : m_self()
        , m_owner(0)
        , m_face_count(0)
      {};

    public:

      vertex_handle  get_handle()     const { return m_self; };
      mesh_type    * get_owner()      const { return m_owner; };
      unsigned int   get_face_count() const { return m_face_count; };

    private:

      friend class trimesh_core_access;
      void set_handle(vertex_handle v) { m_self = v; };
      void set_owner(mesh_type * owner) { m_owner = owner; };

    };

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_VERTEX_H
#endif
