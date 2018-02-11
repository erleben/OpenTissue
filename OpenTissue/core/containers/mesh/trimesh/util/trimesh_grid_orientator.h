#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_GRID_ORIENTATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_GRID_ORIENTATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/trimesh/trimesh.h>
#include <OpenTissue/core/containers/grid/util/grid_gradient_at_point.h>
#include <OpenTissue/core/containers/mesh/trimesh/util/trimesh_compute_face_normal.h>

namespace OpenTissue
{
  namespace trimesh
  {

    template<
        typename M
      , typename V
      , typename F
      , template <typename, typename> class K
      , typename grid_type
    >
    void grid_orientator(grid_type const & grid, TriMesh<M,V,F,K> & mesh, bool same_direction = true)
    {
      typedef          TriMesh<M,V,F,K>                               mesh_type;
      typedef typename mesh_type::face_iterator                       face_iterator;
      typedef typename mesh_type::vertex_iterator                     vertex_iterator;
      typedef typename mesh_type::vertex_handle                       vertex_handle;
      typedef typename mesh_type::math_types                          math_types;
      typedef typename math_types::vector3_type                       vector3_type;
      typedef typename math_types::real_type                          real_type;


      face_iterator end = mesh.face_end();
      face_iterator f   = mesh.face_begin();
      for (;f!=end;++f)
      {
        vertex_iterator v0 = f->get_vertex0_iterator();
        vertex_iterator v1 = f->get_vertex1_iterator();
        vertex_iterator v2 = f->get_vertex2_iterator();
        vector3_type normal;
        compute_face_normal(*f,normal);
        vector3_type grad0 = OpenTissue::grid::gradient_at_point(grid, v0->m_coord);
        vector3_type grad1 = OpenTissue::grid::gradient_at_point(grid, v1->m_coord);
        vector3_type grad2 = OpenTissue::grid::gradient_at_point(grid, v2->m_coord);
        vector3_type avg_grad = (grad0 + grad1 + grad2)/static_cast<real_type>(3.0);
        real_type g_dot_n = avg_grad * normal;
        bool reorder = false;
        if (same_direction &&  g_dot_n < static_cast<real_type>(0.0) )
          reorder = true;
        if (!same_direction &&  g_dot_n > static_cast<real_type>(0.0) )
          reorder = true;
        if(reorder)
        {
          vertex_handle h0 = f->get_vertex0_handle();
          vertex_handle h1 = f->get_vertex1_handle();
          vertex_handle h2 = f->get_vertex2_handle();
          trimesh_core_access::set_vertex0_handle(f,h1);
          trimesh_core_access::set_vertex0_handle(f,h0);
          trimesh_core_access::set_vertex0_handle(f,h2);
        }
      }
    }

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_UTIL_TRIMESH_GRID_ORIENTATOR_H
#endif
