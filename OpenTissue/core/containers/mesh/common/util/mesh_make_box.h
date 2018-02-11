#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_BOX_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_BOX_H
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
  namespace mesh
  {

    /**
    * Box Generation.
    * The method generates an axed aligned box with origo at its center.
    *
    * @param length  The size of the x-side of the box.
    * @param height  The size of the y-side of the box.
    * @param depth   The size of the z-side of the box.
    *
    * @return        If succesful a mesh representating
    *                a box otherwise null.
    */
    template<typename real_type,typename mesh_type>
    bool make_box(real_type const & length,real_type const & height,real_type const & depth,mesh_type & mesh)
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;

      assert(length>=0 || !"Box did not have valid dimensions");
      assert(height>=0 || !"Box did not have valid dimensions");
      assert(depth >=0 || !"Box did not have valid dimensions");

      mesh.clear();

      real_type x = length/static_cast<real_type>(2.0);
      real_type y = height/static_cast<real_type>(2.0);
      real_type z = depth/static_cast<real_type>(2.0);

      vector3_type cl0( x, y,-z);
      vector3_type cl1(-x, y,-z);
      vector3_type cl2(-x,-y,-z);
      vector3_type cl3( x,-y,-z);
      vector3_type cu0( x, y, z);
      vector3_type cu1(-x, y, z);
      vector3_type cu2(-x,-y, z);
      vector3_type cu3( x,-y, z);

      typename mesh_type::vertex_handle l0 = mesh.add_vertex(cl0);
      typename mesh_type::vertex_handle l1 = mesh.add_vertex(cl1);
      typename mesh_type::vertex_handle l2 = mesh.add_vertex(cl2);
      typename mesh_type::vertex_handle l3 = mesh.add_vertex(cl3);

      typename mesh_type::vertex_handle u0 = mesh.add_vertex(cu0);
      typename mesh_type::vertex_handle u1 = mesh.add_vertex(cu1);
      typename mesh_type::vertex_handle u2 = mesh.add_vertex(cu2);
      typename mesh_type::vertex_handle u3 = mesh.add_vertex(cu3);

      mesh.add_face(l1,l0,l2);
      mesh.add_face(l2,l0,l3);
      mesh.add_face(u3,u0,u2);
      mesh.add_face(u2,u0,u1);
      mesh.add_face(l3,l0,u3);
      mesh.add_face(u3,l0,u0);
      mesh.add_face(u1,l1,u2);
      mesh.add_face(u2,l1,l2);
      mesh.add_face(l0,l1,u0);
      mesh.add_face(u0,l1,u1);
      mesh.add_face(l2,l3,u2);
      mesh.add_face(u2,l3,u3);
      {
        typename mesh_type::vertex_iterator vl0 = mesh.get_vertex_iterator(l0);
        vl0->m_normal = normalize(vl0->m_coord);
        vl0->m_u = 0.0;
        vl0->m_v = 0.0;
        vl0->m_color = vector3_type(0.0,0.0,0.0);

        typename mesh_type::vertex_iterator vl1 = mesh.get_vertex_iterator(l1);
        vl1->m_normal = normalize(vl1->m_coord);
        vl1->m_u = 1.0;
        vl1->m_v = 0.0;
        vl1->m_color = vector3_type(1.0,0.0,0.0);

        typename mesh_type::vertex_iterator vl2 = mesh.get_vertex_iterator(l2);
        vl2->m_normal = normalize(vl2->m_coord);
        vl2->m_u = 1.0;
        vl2->m_v = 1.0;
        vl2->m_color = vector3_type(1.0,1.0,0.0);

        typename mesh_type::vertex_iterator vl3 = mesh.get_vertex_iterator(l3);
        vl3->m_normal = normalize(vl3->m_coord);
        vl3->m_u = 0.0;
        vl3->m_v = 1.0;
        vl3->m_color = vector3_type(0.0,1.0,0.0);

        typename mesh_type::vertex_iterator vu0 = mesh.get_vertex_iterator(u0);
        vu0->m_normal = normalize(vu0->m_coord);
        vu0->m_u = 0.0;
        vu0->m_v = 0.0;
        vu0->m_color = vector3_type(0.0,0.0,1.0);

        typename mesh_type::vertex_iterator vu1 = mesh.get_vertex_iterator(u1);
        vu1->m_normal = normalize(vu1->m_coord);
        vu1->m_u = 1.0;
        vu1->m_v = 0.0;
        vu1->m_color = vector3_type(1.0,0.0,1.0);

        typename mesh_type::vertex_iterator vu2 = mesh.get_vertex_iterator(u2);
        vu2->m_normal = normalize(vu2->m_coord);
        vu2->m_u = 1.0;
        vu2->m_v = 1.0;
        vu2->m_color = vector3_type(1.0,1.0,1.0);

        typename mesh_type::vertex_iterator vu3 = mesh.get_vertex_iterator(u3);
        vu3->m_normal = normalize(vu3->m_coord);
        vu3->m_u = 0.0;
        vu3->m_v = 1.0;
        vu3->m_color = vector3_type(0.0,1.0,1.0);
      }
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_BOX_H
#endif
