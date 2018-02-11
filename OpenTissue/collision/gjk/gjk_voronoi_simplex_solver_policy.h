#ifndef OPENTISSUE_COLLISION_GJK_GJK_VORONOI_SIMPLEX_SOLVER_POLICY_H
#define OPENTISSUE_COLLISION_GJK_GJK_VORONOI_SIMPLEX_SOLVER_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/gjk/gjk_simplex.h>
#include <OpenTissue/collision/gjk/gjk_reduce_edge.h>
#include <OpenTissue/collision/gjk/gjk_reduce_triangle.h>
#include <OpenTissue/collision/gjk/gjk_reduce_tetrahedron.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {

    /**
    * Voronoi Simplex Policy.
    * This class implements a policy for how GJK-type of algorithms
    * make changes to the underlying simplex approximation.
    *
    * This particular policy updates and make changes to the simplex
    * based on voronoi region case-by-case analysis.
    */
    class VoronoiSimplexSolverPolicy
    {
    public:

      /**
      * Reduce Simplex.
      * Compute the point, v, on the simplex that is closest to the origin and
      * Reduce simplex to lowest dimensional face on the boundary
      * containing the closest point.
      *
      * @param S         Upon invocation this argument holds the initial
      *                  simplex and upon return the argument holds the
      *                  resulting reduced simplex.
      *
      * @param a         Upon return this argument holds the corresponding
      *                  closest point on the set A.
      *
      * @param b         Upon return this argument holds the corresponding
      *                  closest point on the set B.
      *
      * @return          The closest point, v, on the simplex to the origin.
      */
      template<typename V>
      static V reduce_simplex( Simplex<V> & S, V & a, V & b )
      {
        typedef typename V::value_traits   value_traits;

        V const p = V( value_traits::zero(), value_traits::zero(), value_traits::zero() );

        switch( dimension( S ) )
        {
        case 1:
          {
            // Nothing to do, a vertex can not be reduced!
            int bit_A    = 0;
            size_t idx_A = 0;

            get_used_indices( S.m_bitmask, idx_A, bit_A );

            S.m_bitmask = bit_A;
            S.m_w[idx_A] = value_traits::one();
          }
          break;
        case 2: 
          detail::reduce_edge(p, S); 
          break;
        case 3: 
          detail::reduce_triangle(p, S); 
          break;
        case 4: 
          detail::reduce_tetrahedron(p, S); 
          break;
        default:
          assert(false || !"reduce_simplex(): can not reduce simplex of that size");
          break;
        };

        // Now compute the actual closest points based on the bary-centric coordinates.
        a.clear();
        b.clear();
        int used_bit = 1;
        for(size_t i=0u; i<4u; ++i)
        {
          if(S.m_bitmask & used_bit)
          {
            a += S.m_w[i]*S.m_a[i];
            b += S.m_w[i]*S.m_b[i];
          }
          used_bit <<= 1;
        }
        return (a-b);
      }

    };

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_VORONOI_SIMPLEX_SOLVER_POLICY_H
#endif
