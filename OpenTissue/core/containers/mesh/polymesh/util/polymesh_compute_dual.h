#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DUAL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DUAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_center.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>
#include <boost/property_map/vector_property_map.hpp>
#include <list>
#include <stdexcept> // needed for std::invalid_argument

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * A simple vertex policy for computing the coordinates and
    * normals of the vertices in a dual mesh.
    */
    struct center_vertex_policy
    {

      template<typename face_type, typename vector3_type>
      static void compute_position( face_type const & face, vector3_type & coord )
      {
        mesh::compute_face_center( face, coord );
      }

      template<typename face_type, typename vector3_type>
      static void compute_normal( face_type const & face, vector3_type & normal )
      {
        normal.clear();
      }

    };


    /**
    * PolyMesh Compute Dual Mesh.
    *
    * @param original        The orginal mesh.
    * @param dual            The dual mesh. Coordinates and normals of the vertices in the new
    *                        mesh are computed using the vertex_policy template argument. The
    *                        policy is specfied using a tag argumnt.
    * @param vertex_policy   A policy used to compute positions and normals of new vertices.
    */
    template< typename mesh_type, typename vertex_policy >
    void compute_dual(mesh_type const & original, mesh_type & dual, vertex_policy const & /*tag*/  )
    {
      typedef typename mesh_type::math_types                    math_types;
      typedef typename math_types::vector3_type                 vector3_type;
      typedef typename mesh_type::vertex_handle                 vertex_handle;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_vertex_iterator         const_vertex_iterator;
      typedef typename mesh_type::const_vertex_face_circulator  const_vertex_face_circulator;

      if(&original == &dual)
        throw std::invalid_argument("polymesh::compute_dual(): original and dual meshes must not be the same");

      dual.clear();

      boost::vector_property_map<vertex_handle> handles;
      {
        const_face_iterator face = original.face_begin();
        const_face_iterator fend = original.face_end();
        for(;face!=fend;++face)
        {
          vector3_type coord;

          vertex_policy::compute_position( *face, coord );

          vertex_handle h = dual.add_vertex( coord );
          handles[ face->get_handle().get_idx() ] = h;

          vertex_policy::compute_normal( *face, dual.get_vertex_iterator(h)->m_normal );          
        }
      }
      {
        const_vertex_iterator vertex = original.vertex_begin();
        const_vertex_iterator vend   = original.vertex_end();
        for(;vertex!=vend;++vertex)
        {

          if( is_boundary( *vertex ) )
            continue;

          std::list<vertex_handle> vertices;

          const_vertex_face_circulator face(*vertex), fend;          
          for(;face!=fend;++face)
            vertices.push_back( handles[face->get_handle().get_idx()] );

          if(vertices.size()<3)
            continue;

          dual.add_face(vertices.begin(),vertices.end());
        }
      }

    }



    /**
    * PolyMesh Compute Dual Mesh.
    *
    * @param original    The orginal mesh.
    * @param dual        The dual mesh. Coordinates and normals of the vertices in the new
    *                    mesh are computed using the vertex_policy template argument. The
    *                    policy is specfied using a tag argumnt.
    *
    */
    template< typename mesh_type >
    void compute_dual(mesh_type const & original, mesh_type & dual  )
    {
      compute_dual( original, dual, center_vertex_policy() );
    }




  } // namespace polymesh
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DUAL_H
#endif
