#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONVEX_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONVEX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_dihedral_angle.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_center.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_plane.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>
#include <OpenTissue/core/geometry/geometry_plane.h>

#include <functional> // for std::greater

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type,typename real_type>
    bool is_convex(PolyMeshHalfEdge<mesh_type> const & e,real_type const & tolerance)
    {
      real_type radian;
      compute_dihedral_angle(e,radian);
      if( radian <= -tolerance )
        return false;
      return true;
    }

    template<typename mesh_type>
    bool is_convex(PolyMeshHalfEdge<mesh_type> const & e)  
    {    
      typedef typename mesh_type::math_types::value_traits  value_traits;
      return is_convex(e,value_traits::zero());  
    }

    template<typename mesh_type,typename real_type>
    bool is_convex(PolyMeshFace<mesh_type> const & face, real_type const & tolerance)
    {
      typedef typename mesh_type::face_face_circulator face_face_circulator;

      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::vector3_type              vector3_type;
      typedef geometry::Plane<math_types>                    plane_type;

      if(valency(face)==0)
      {
        std::cout << "is_convex(face): No border!" << std::endl;
        return false;
      }
      plane_type             plane;
      vector3_type           center;

      compute_face_center(face,center);

      face_face_circulator   twin(face),end;
      for(;twin!=end;++twin)
      {
        compute_face_plane( *twin, plane );
        if(plane.signed_distance(center) > tolerance)
          return false;
      }
      return true;
    }

    template<typename mesh_type>
    bool is_convex(PolyMeshFace<mesh_type> const & face)  
    {    
      typedef typename mesh_type::math_types::value_traits  value_traits;

      return is_convex(face,value_traits::zero());  
    }

    template<typename mesh_type,typename real_type>
    bool is_convex(PolyMeshVertex<mesh_type> const & v,real_type const & tolerance)
    {
      typedef typename mesh_type::vertex_halfedge_circulator  vertex_halfedge_circulator;

      if(valency(v)==0)
        return false;

      vertex_halfedge_circulator h(v), end;
      for(;h!=end;++h)
      {
        if( ! is_convex(*h,tolerance) )
          return false;
      }
      return true;
    }

    template<typename mesh_type>
    bool is_convex(PolyMeshVertex<mesh_type> const & v) 
    {
      typedef typename mesh_type::math_types::value_traits  value_traits;
      return is_convex(v, value_traits::zero()); 
    }

    template< typename mesh_type, typename real_type>
    bool is_convex( mesh_type const & mesh, real_type const & tolerance)
    {
      typename mesh_type::const_face_iterator end   = mesh.face_end();
      typename mesh_type::const_face_iterator f     = mesh.face_begin();
      for(;f!=end;++f)
      {
        if(! is_convex( *f, tolerance ) )
          return false;
      }
      return true;
    }

    template< typename mesh_type >
    bool is_convex( mesh_type const & mesh)
    {
      typedef typename mesh_type::math_types::value_traits  value_traits;
      return is_convex(mesh, value_traits::zero());
    }

  } // namespace polymesh

} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONVEX_H
#endif
