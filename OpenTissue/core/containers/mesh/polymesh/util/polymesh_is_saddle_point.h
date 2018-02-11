#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SADDLE_POINT_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SADDLE_POINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_convex.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type,typename real_type>
    bool is_saddle_point(PolyMeshVertex<mesh_type> const & v,real_type const & tolerance)
    {
      typedef typename mesh_type::vertex_halfedge_circulator  vertex_halfedge_circulator;

      if(valency(v)==0)
        return false;

      bool convex = false;
      bool concave = false;

      vertex_halfedge_circulator h(v), end;
      for(;h!=end;++h)
      {
        if( is_convex( *h, tolerance ) )
          convex = true;
        else
          concave = true;
        if(convex && concave)
          return true;
      }
      return false;
    }

    template<typename mesh_type>
    bool is_saddle_point(PolyMeshVertex<mesh_type> const & v) 
    {
      typedef typename mesh_type::math_types     math_types;
      typedef typename math_types::value_traits  value_traits;

      return is_saddle_point(v,value_traits::zero()); 
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_IS_SADDLE_POINT_H
#endif
