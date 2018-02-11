#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_SUBDIVIDE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_SUBDIVIDE_H
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
    * Polymesh Subdivide.
    * Subdivides all faces with an area larger than the specified threshold.
    *
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
    * @param mesh       The mesh that should be subdivided.
    * @param max_area   The area threshold value.
    *
    * @return           The number of subdivided faces.
    */
    template< typename mesh_type >
    unsigned int subdivide( mesh_type & mesh, double max_area = 0.01  )
    {
      typedef typename mesh_type::vertex_handle  vertex_handle;
      typedef typename mesh_type::edge_handle    edge_handle;

      typedef typename mesh_type::face_iterator            face_iterator;
      typedef typename mesh_type::face_vertex_circulator   face_vertex_circulator;
      typedef typename mesh_type::face_halfedge_circulator face_halfedge_circulator;
      typedef typename mesh_type::halfedge_iterator        halfedge_iterator;
      typedef typename mesh_type::vertex_iterator          vertex_iterator;
      typedef typename mesh_type::edge_iterator            edge_iterator;

      typedef typename mesh_type::math_types               math_types;
      typedef typename math_types::value_traits            value_traits;
      typedef typename math_types::vector3_type            vector3_type;
      typedef typename math_types::real_type               real_type;

      std::map<edge_handle, vertex_handle>  lut;

      real_type max_area_sqr = static_cast<real_type>(  max_area*max_area );

      unsigned int cnt = 0; ///< The number of faces that were subdivided.

      for(face_iterator f = mesh.face_begin();f!=mesh.face_end();++f)
      {
        face_halfedge_circulator h0(*f);
        face_halfedge_circulator h1(*f);++h1;
        face_halfedge_circulator h2(*f);--h2;

        edge_handle e0 = h0->get_edge_handle();
        edge_handle e1 = h1->get_edge_handle();
        edge_handle e2 = h2->get_edge_handle();

        vertex_handle v0 = h0->get_origin_handle();
        vertex_handle v1 = h1->get_origin_handle();
        vertex_handle v2 = h2->get_origin_handle();

        vector3_type p0 = h0->get_origin_iterator()->m_coord;
        vector3_type p1 = h1->get_origin_iterator()->m_coord;
        vector3_type p2 = h2->get_origin_iterator()->m_coord;

        vector3_type u1       = p2 - p1;
        vector3_type u2       = p1 - p0;
        vector3_type u1xu2    = u1 % u2;
        real_type area_sqr = u1xu2*u1xu2;
        if(area_sqr > max_area_sqr)
        {
          ++cnt;
          if(lut.find(e0) == lut.end())
            lut[e0] = mesh.add_vertex((p0+p1)*.5);
          if(lut.find(e1) == lut.end())
            lut[e1] = mesh.add_vertex((p1+p2)*.5);
          if(lut.find(e2) == lut.end())
            lut[e2] = mesh.add_vertex((p2+p0)*.5);
        }
      }

      for(face_iterator f = mesh.face_begin();f!=mesh.face_end();)
      {
        face_halfedge_circulator h0(*f);
        face_halfedge_circulator h1(*f);++h1;
        face_halfedge_circulator h2(*f);--h2;

        edge_handle e0 = h0->get_edge_handle();
        edge_handle e1 = h1->get_edge_handle();
        edge_handle e2 = h2->get_edge_handle();

        vertex_handle v0 = h0->get_origin_handle();
        vertex_handle v1 = h1->get_origin_handle();
        vertex_handle v2 = h2->get_origin_handle();

        vertex_handle v01 = lut[e0];
        vertex_handle v12 = lut[e1];
        vertex_handle v20 = lut[e2];

        if(      !v01.is_null()  &&  v12.is_null() &&  v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face( v0,  v01, v2 );
          mesh.add_face( v01,  v1, v2 );
        }
        else if(  v01.is_null()  && !v12.is_null() &&  v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face( v0,  v1, v12 );
          mesh.add_face( v0, v12,  v2 );
        }
        else if(  v01.is_null()  &&  v12.is_null() && !v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face( v0, v1, v20);
          mesh.add_face( v1, v2, v20);
        }
        else if(  v01.is_null()  && !v12.is_null() && !v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face(   v2, v20, v12 );
          mesh.add_face(   v0,  v1, v20 );
          mesh.add_face(  v20,  v1, v12 );
        }
        else if( !v01.is_null()  &&  v12.is_null() && !v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face(   v0, v01, v20  );
          mesh.add_face(   v01,  v1, v2  );
          mesh.add_face(   v2,  v20, v01 );
        }
        else if( !v01.is_null()  && !v12.is_null() &&  v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face(   v1, v12, v01 );
          mesh.add_face(   v0,  v01, v2 );
          mesh.add_face(  v01,  v12, v2 );
        }
        else if( !v01.is_null()  && !v12.is_null() && !v20.is_null())
        {
          face_iterator tmp = f;++f;
          mesh.remove_face( tmp->get_handle());
          mesh.add_face(  v0, v01, v20 );
          mesh.add_face(  v1, v12, v01 );
          mesh.add_face(  v2, v20, v12 );
          mesh.add_face( v01, v12, v20 );
        }
        else
        {
          ++f;
        }
      }
      return cnt;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_SUBDIVIDE_H
#endif
