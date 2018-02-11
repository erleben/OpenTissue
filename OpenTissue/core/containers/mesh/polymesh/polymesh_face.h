#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_FACE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_FACE_H
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
  class PolyMeshFace : public polymesh_type_::face_traits
  {
  public:

    typedef polymesh_type_                         mesh_type;
    typedef typename mesh_type::halfedge_handle    halfedge_handle;
    typedef typename mesh_type::face_handle        face_handle;
    typedef typename mesh_type::halfedge_iterator  halfedge_iterator;

  private:

    face_handle        m_self;
    mesh_type       *  m_owner;
    halfedge_handle    m_border_halfedge;

  public:

    PolyMeshFace()
      : m_self()
      , m_owner(0)
      , m_border_halfedge()
    {}

  public:

    face_handle       get_handle() const { return m_self; }
    mesh_type     *   get_owner() const { return m_owner; }
    halfedge_handle   get_border_halfedge_handle() const { return m_border_halfedge; }
    halfedge_iterator get_border_halfedge_iterator() const { return m_owner->get_halfedge_iterator(m_border_halfedge); }

  private:

    friend class polymesh_core_access;
    void set_handle(face_handle f) { m_self = f; }
    void set_owner(mesh_type * owner) { m_owner = owner; }
    void set_border_halfedge_handle(halfedge_handle h) { m_border_halfedge = h; }

  };

} // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_FACE_H
#endif
