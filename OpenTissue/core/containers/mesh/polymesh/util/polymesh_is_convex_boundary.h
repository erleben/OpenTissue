#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONVEX_BOUNDARY_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONVEX_BOUNDARY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_face_normal.h>


namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type>
    bool is_convex_boundary(PolyMeshFace<mesh_type> const & f)
    {
      typedef typename mesh_type::face_halfedge_circulator   face_halfedge_circulator;
      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::vector3_type              vector3_type;

      if(valency(f)==0)
      {
        std::cout << "is_convex_boundary(face): No border!" << std::endl;
        return false;
      }

      vector3_type n;
      compute_face_normal(f, n);

      face_halfedge_circulator   cur(f),end;
      face_halfedge_circulator   next(f);  ++next;

      vector3_type u1 = cur->get_destination_iterator()->m_coord  - cur->get_origin_iterator()->m_coord;

      for(;cur!=end;++cur,++next)
      {
        vector3_type u2 = next->get_destination_iterator()->m_coord  - next->get_origin_iterator()->m_coord;
        if( (n*cross(u1 , u2)) <0)
          return false;
        u1 = u2;
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONVEX_BOUNDARY_H
#endif
