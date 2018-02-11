#ifndef OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_VERTEX_EDGE_VORONOI_PLANE_H
#define OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_VERTEX_EDGE_VORONOI_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {

    namespace detail
    {

      /**
      * Test if point is outside a vertex edge voronoi plane.
      * The vertex edge voronoi plane is defined such that the plane normal is given
      * as the unit vector of the vector \fA-B\f and the point \fA\f is defined to
      * lie in the plane, and the point \fB\f behind the plane.
      *
      * @param p    The point that should be tested.
      * @param A    The first vertex of the edge.
      * @param B    The second vertex of the edge.
      *
      * @return     If point is outside or on plane then return value is true otherwise it is false.
      */
      template< typename V >
      inline bool outside_vertex_edge_voronoi_plane( 
        V const & p
        , V const & A
        , V const & B
        )
      {
        typedef typename V::value_traits    value_traits;
        typedef typename V::value_type      T;

        V const n = (A-B);
        assert( dot( n, n ) > value_traits::zero() || !"outside_vertex_edge_voronoi_plane(): Degenerate edge encountered");

        T const sign_p = dot( n, (p-A) );
        assert( is_number( sign_p ) || !"outside_vertex_edge_voronoi_plane(): Not a Number encountered");

        return sign_p >= value_traits::zero();
      }


    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_VERTEX_EDGE_VORONOI_PLANE_H
#endif
