#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SUBDIVIDE2_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SUBDIVIDE2_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

#include <map>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Subdivide a Face into smaller faces.
    * Subdivides face, by inserting new vertices at edge midpoints.
    *
    * Before
    *
    *             +
    *            / \
    *           /   \
    *          /     \
    *         /       \
    *        /         \
    *       +-----------+
    *      / \         / \
    *     /   \   f   /   \
    *    /     \     /     \
    *   /       \   /       \
    *  /         \ /         \
    * +-----------+----------+
    *
    * After
    *             +
    *            /|\
    *           / | \
    *          /  |  \
    *         /   |   \
    *        /    |    \
    *       +-----+-----+
    *      / \   / \   / \
    *     /   \ /   \ /   \
    *    /     +---- +     \
    *   /   /   \   /   \   \
    *  / /       \ /      \  \
    * +-----------+----------+
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    * @param f
    *
    * @return
    */
    template<typename mesh_type>
    bool face_subdivide2(PolyMeshFace<mesh_type> & f)
    {
      typedef typename mesh_type::face_vertex_circulator        face_vertex_circulator;
      typedef typename mesh_type::face_iterator                 face_iterator;
      typedef typename mesh_type::vertex_handle                 vertex_handle;

      typedef typename mesh_type::math_types                    math_types;
      typedef typename math_types::vector3_type                 vector3_type;

      mesh_type * owner = f.get_owner();
      if(owner==0)
      {
        assert(!"face_subdivide2(): No mesh owner!");
        return false;
      }
      typedef typename mesh_type::face_halfedge_circulator        face_halfedge_circulator;


      if(is_boundary(f))      return false;
      face_halfedge_circulator h0(f);
      face_halfedge_circulator h1(f);++h1;
      face_halfedge_circulator h2(f);--h2;
      if(is_boundary(*h0))      return false;
      if(is_boundary(*h1))      return false;
      if(is_boundary(*h2))      return false;
      face_iterator n0 = h0->get_twin_iterator()->get_face_iterator();
      face_iterator n1 = h1->get_twin_iterator()->get_face_iterator(); 
      face_iterator n2 = h2->get_twin_iterator()->get_face_iterator();

      vertex_handle v0 = h0->get_origin_handle();
      vertex_handle v1 = h0->get_twin_iterator()->get_next_iterator()->get_destination_handle();
      vertex_handle v2 = h1->get_origin_handle();
      vertex_handle v3 = h1->get_twin_iterator()->get_next_iterator()->get_destination_handle();
      vertex_handle v4 = h2->get_origin_handle();
      vertex_handle v5 = h2->get_twin_iterator()->get_next_iterator()->get_destination_handle();

      vector3_type p0 = h0->get_origin_iterator()->m_coord;
      vector3_type p1 = h1->get_origin_iterator()->m_coord;
      vector3_type p2 = h2->get_origin_iterator()->m_coord;

      owner->remove_face( f.get_handle());
      owner->remove_face(n0->get_handle());
      owner->remove_face(n1->get_handle());
      owner->remove_face(n2->get_handle());
      vertex_handle v6 = owner->add_vertex((p0+p1)*.5);
      vertex_handle v7 = owner->add_vertex((p1+p2)*.5);
      vertex_handle v8 = owner->add_vertex((p2+p0)*.5);

      owner->add_face( v0, v1 , v6);
      owner->add_face( v1, v2 , v6);
      owner->add_face( v2, v3 , v7);
      owner->add_face( v3, v4 , v7);
      owner->add_face( v4, v5 , v8);
      owner->add_face( v5, v0 , v8);
      owner->add_face( v0, v6 , v8);
      owner->add_face( v6, v2 , v7);
      owner->add_face( v7, v4 , v8);
      owner->add_face( v6, v7 , v8);

      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SUBDIVIDE2_H
#endif
