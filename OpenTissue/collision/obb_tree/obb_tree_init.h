#ifndef OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_INIT_H
#define OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_INIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/obb_tree/obb_tree_types.h>

#include <vector>

namespace OpenTissue
{
  namespace obb_tree
  {

    /**
     * Initialize Oriented Bounding Box (OBB) volume hierarchy.
     *
     * @param mesh     The mesh containing the geometry of the hiearchy.
     * @param tree     Upon return this argument holds the generated hiearchy.
     * @param degree   The desired degree of the resulting hierarchy. Legal
     *                 values are 2, 4, and 8. Default value is 2.
     */
    template<typename obb_tree_types>
    void init(
        typename obb_tree_types::mesh_type & mesh
      , typename obb_tree_types::bvh_type & tree
      , size_t const & degree = 2
      )
    {
      typedef typename obb_tree_types::index_type               index_type;
      typedef typename obb_tree_types::construtor_type constructor_type;
      typedef typename obb_tree_types::face_ptr_type            face_ptr_type;

      typedef typename obb_tree_types::mesh_type       mesh_type;
      typedef typename mesh_type::face_iterator                 face_iterator;
      typedef typename mesh_type::face_vertex_circulator        face_vertex_circulator;


      index_type F = mesh.size_faces();
      std::vector<face_ptr_type> geometry(F,0);

      face_iterator f     = mesh.face_begin();
      face_iterator f_end = mesh.face_end();

      for(index_type i=0; f!=f_end; ++f,++i)
      {
        geometry[i] = &(*f);

        if(valency(*f)!=3)
          throw std::logic_error("Only triangle faces are supported");

        face_vertex_circulator v(*f);
        f->m_v0 = &(v->m_coord);
        f->m_n0 = &(v->m_normal);
        ++v;
        f->m_v1 = &(v->m_coord);
        f->m_n1 = &(v->m_normal);
        ++v;
        f->m_v2 = &(v->m_coord);
        f->m_n2 = &(v->m_normal);
      }
      tree.clear();
      constructor_type constructor;
      constructor.set_degree( degree );
      constructor.run(geometry.begin(),geometry.end(),tree);
    }

  }// namespace obb_tree

} // namespace OpenTissue

//OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_INIT_H
#endif
