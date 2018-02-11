#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONCAVE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONCAVE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_dihedral_angle.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_convex.h>

#include <functional> // std::less

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type,typename real_type>
    bool is_concave(PolyMeshHalfEdge<mesh_type> const & e,real_type const & tolerance)
    {
      real_type radian;
      compute_dihedral_angle(e,radian);
      if( radian >= tolerance )
        return false;
      return true;
    }

    template<typename mesh_type>
    bool is_concave(PolyMeshHalfEdge<mesh_type> const & e)  
    {
      typedef typename mesh_type::math_types::value_traits  value_traits;

      return is_concave(e,value_traits::zero());  
    }

    template<typename mesh_type,typename real_type>
    bool is_concave(PolyMeshVertex<mesh_type> const & v,real_type const & tolerance)
    {
      typedef typename mesh_type::vertex_halfedge_circulator  vertex_halfedge_circulator;
      if(valency(v)==0)
        return false;
      vertex_halfedge_circulator h(v), end;
      for(;h!=end;++h)
      {
        if( !is_concave(*h,tolerance) )
          return false;
      }
      return true;
    }

    template<typename mesh_type>
    bool is_concave(PolyMeshVertex<mesh_type> const & v) 
    {
      typedef typename mesh_type::math_types::value_traits  value_traits;

      return is_concave(v,value_traits::zero()); 
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_CONCAVE_H
#endif
