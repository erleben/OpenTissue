#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SUBDIVIDE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SUBDIVIDE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_center.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Subdivide a Face into smaller faces.
    * This subdivision is based on adding a new vertex at the midpoint of the face.
    *
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
    *
    *
    *             +
    *            / \
    *           /   \
    *          /     \
    *         /       \
    *        /         \
    *       +-----------+
    *      / \\       // \
    *     /   \ \   / /   \
    *    /     \  +  /     \
    *   /       \ | /       \
    *  /         \|/         \
    * +-----------+----------+
    *
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    * @param f
    *
    * @return
    */
    template<typename mesh_type>
    bool face_subdivide(PolyMeshFace<mesh_type> & f)
    {
      typedef typename mesh_type::face_vertex_circulator        face_vertex_circulator;
      typedef typename mesh_type::vertex_handle                 vertex_handle;
      typedef typename mesh_type::math_types                    math_types;
      typedef typename math_types::vector3_type                 vector3_type;

      mesh_type * owner = f.get_owner();
      if(owner==0)
      {
        assert(!"face_subdivide(): No mesh owner!");
        return false;
      }
      int i,j;
      int n = valency(f);
      std::vector<vertex_handle> boundary(n);
      face_vertex_circulator v(f);
      for(i=0;i<n;++v,++i)
        boundary[i] = v->get_handle();
      vector3_type center(0,0,0);
      mesh::compute_face_center(f,center);
      vertex_handle k = owner->add_vertex(center);
      owner->remove_face(f.get_handle());
      for(i=0,j=1;i<n;++i,j=(j+1)%n)
      {
        owner->add_face( boundary[i],boundary[j],k);
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_FACE_SUBDIVIDE_H
#endif
