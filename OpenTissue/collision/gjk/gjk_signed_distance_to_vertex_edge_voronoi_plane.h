#ifndef OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_VERTEX_EDGE_VORONOI_PLANE_H
#define OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_VERTEX_EDGE_VORONOI_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {
    namespace detail
    {

      /**
      * Signed Distance between a point and a vertex edge voronoi plane.
      * The vertex edge voronoi plane is defined such that the plane normal is given
      * as the unit vector of the vector \fA-B\f and the point \fA\f is defined to
      * lie in the plane, and the point \fB\f behind the plane.
      *
      * @param p    The point that should be tested.
      * @param A    The first vertex of the edge.
      * @param B    The second vertex of the edge.
      *
      * @return     The signed distance of the point p.
      */
      template< typename V >
      inline typename V::value_type signed_distance_to_vertex_edge_voronoi_plane( 
        V const & p
        , V const & A
        , V const & B
        )
      {
        typedef typename V::value_traits    value_traits;
        typedef typename V::value_type      T;

        V m = (A-B);

        assert( dot( m, m ) > value_traits::zero() || !"signed_distance_to_vertex_edge_voronoi_plane(): Degenerate edge encountered");

        V n = unit( m );

        T sign_p = dot( n, (p-A) );

        assert( is_number( sign_p ) || !"signed_distance_to_vertex_edge_voronoi_plane(): Not a Number encountered");

        return sign_p;
      }

    } // namespace detail
  } // namespace gjk
} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_VERTEX_EDGE_VORONOI_PLANE_H
#endif
