#ifndef OPENTISSUE_COLLISION_GJK_GJK_REDUCE_EDGE_H
#define OPENTISSUE_COLLISION_GJK_GJK_REDUCE_EDGE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_barycentric.h>

#include <OpenTissue/collision/gjk/gjk_simplex.h>
#include <OpenTissue/collision/gjk/gjk_outside_vertex_edge_voronoi_plane.h>

namespace OpenTissue
{
  namespace gjk
  {

    namespace detail
    {

      /**
      * Reduce Edge S.
      * This function implements the case where a simplex
      * is an edge. The function will compute the closest
      * point to p on the simplex and try to reduce the simplex
      * to the lowest dimensional face on the boundary of the
      * simplex containing the closest point.
      *
      * @param p         The test point.
      * @param simplex   Initially this argument holds the edge simplex. Upon
      *                  return the argument holds the reduced simplex.
      */
      template< typename V >
      inline void reduce_edge( V const & p, Simplex<V> & S)
      {
        typedef typename V::value_type    T;
        typedef typename V::value_traits  value_traits;
        
        int bit_A = 0;
        int bit_B = 0;
        size_t idx_A = 0u;
        size_t idx_B = 0u;
        get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );
        
        V const & A = S.m_v[idx_A];
        V const & B = S.m_v[idx_B];
        
        bool const outside_AB = outside_vertex_edge_voronoi_plane( p, A, B );
        bool const outside_BA = outside_vertex_edge_voronoi_plane( p, B, A );
        
        if(outside_AB)
        {
          // Simplex is A
          S.m_bitmask = bit_A;
          S.m_v[idx_B].clear();
          S.m_a[idx_B].clear();
          S.m_b[idx_B].clear();
          S.m_w[idx_A] = value_traits::one();
          S.m_w[idx_B] = value_traits::zero();
          return;
        }
        if(outside_BA)
        {
          // Simplex is B
          S.m_bitmask = bit_B;
          S.m_v[idx_A].clear();
          S.m_a[idx_A].clear();
          S.m_b[idx_A].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_B] = value_traits::one();
          return;
        }
        if(!outside_AB && !outside_BA)
        {
          OpenTissue::geometry::barycentric_geometric(A,B,p,S.m_w[idx_A],S.m_w[idx_B]);
          return;
        }

      }

    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_REDUCE_EDGE_H
#endif
