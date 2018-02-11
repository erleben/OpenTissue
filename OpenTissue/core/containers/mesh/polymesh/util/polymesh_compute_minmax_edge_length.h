#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_MIN_MAX_EDGE_LENGTH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_MIN_MAX_EDGE_LENGTH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_direction.h>
#include <boost/cast.hpp> //--- Needed for boost::numeric_cast


namespace OpenTissue
{
  namespace polymesh
  {

    /**
     * Compute Mesh Edge statistics.
     *
     *
     * @param mesh        A reference to a mesh object.
     * @param min_length  Upon return this argument holds the minimum length of the edges.
     * @param max_length  Upon return this argument holds the maximum length of the edges.
     * @param avg_length  Upon return this argument holds the average length of the edges.
     */
    template<typename mesh_type, typename real_type>
    inline void compute_minmax_edge_length( 
        mesh_type const & mesh
      , real_type & min_length
      , real_type & max_length
      , real_type & avg_length
      )
    {
      using std::sqrt;

      typedef typename mesh_type::math_types::vector3_type       vector3_type;
      typedef typename OpenTissue::math::ValueTraits<real_type>  value_traits;

      min_length = value_traits::infinity();
      max_length = value_traits::zero();
      avg_length = value_traits::zero();

      typename mesh_type::const_edge_iterator e    = mesh.edge_begin();
      typename mesh_type::const_edge_iterator end  = mesh.edge_end();
      for(;e!=end;++e)
      {
        vector3_type u;

        OpenTissue::polymesh::compute_edge_direction( *(e->get_halfedge0_iterator()), u);

        real_type edge_length = boost::numeric_cast<real_type>( sqrt( inner_prod( u,u) ) );
        avg_length += edge_length;
        min_length = (edge_length < min_length)?edge_length: min_length;
        max_length = (edge_length > max_length)?edge_length: max_length;
      }
      avg_length /= mesh.size_edges();
    }

  } // namespace polymesh
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_MIN_MAX_EDGE_LENGTH_H
#endif
