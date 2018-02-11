#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_PLANAR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_PLANAR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_dihedral_angle.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_face_normal.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type,typename real_type>
    bool is_planar(PolyMeshHalfEdge<mesh_type> const & e,real_type const & tolerance)
    {
      real_type radian;
      compute_dihedral_angle(e,radian);
      if(tolerance < radian || radian < -tolerance)
        return false;
      return true;
    }

    template<typename mesh_type>
    bool is_planar(PolyMeshHalfEdge<mesh_type> const & e)  
    {    
      typedef typename mesh_type::math_types     math_types;
      typedef typename math_types::value_traits  value_traits;
      return is_planar(e,value_traits::zero());  
    }

    template<typename mesh_type,typename real_type>
    bool is_planar(PolyMeshFace<mesh_type> const & f, real_type const & tolerance )
    {
      typedef typename mesh_type::face_vertex_circulator     face_vertex_circulator;
      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::vector3_type              vector3_type;

      if(valency(f)==0)
      {
        std::cout << "is_planar(face): No border!" << std::endl;
        return false;
      }
      vector3_type n;
      compute_face_normal( f, n );
      face_vertex_circulator v(f),end;
      real_type min_proj = n*v->m_coord;
      real_type max_proj = min_proj;
      ++v;
      for(;v!=end;++v)
      {
        real_type proj = n*v->m_coord;
        min_proj = std::min(min_proj,proj);
        max_proj = std::max(max_proj,proj);
      }
      if( (max_proj-min_proj)>tolerance)
        return false;
      return true;
    }

    template<typename mesh_type>
    bool is_planar(PolyMeshFace<mesh_type> const & f)  
    {    
      typedef typename mesh_type::math_types     math_types;
      typedef typename math_types::value_traits  value_traits;
      return is_planar(f, value_traits::zero() );  
    }

    template<typename mesh_type,typename real_type>
    bool is_planar(PolyMeshVertex<mesh_type> const & v,real_type const & tolerance)
    {
      typedef typename mesh_type::vertex_halfedge_circulator   vertex_halfedge_circulator;
      if(valency(v)==0)
        return true;
      vertex_halfedge_circulator h(v), end;
      for(;h!=end;++h)
      {
        if(! is_planar(*h,tolerance) )
          return false;
      }
      return true;
    }

    template<typename mesh_type>
    bool is_planar(PolyMeshVertex<mesh_type> const & v)
    {
      typedef typename mesh_type::math_types     math_types;
      typedef typename math_types::value_traits  value_traits;

      return is_planar(v,value_traits::zero()); 
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_PLANAR_H
#endif
