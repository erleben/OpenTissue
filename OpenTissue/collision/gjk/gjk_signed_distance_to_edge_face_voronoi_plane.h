#ifndef OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_EDGE_FACE_VORONOI_PLANE_H
#define OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_EDGE_FACE_VORONOI_PLANE_H
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
      * Signed Distance between a point and a Edge Face Voronoi Plane.
      *
      * @param p A point which we want to know whether it lies outside a face voronoi
      * region. A face-voronoi region is given by a collection of edge-face voronoi
      * planes and the face-plane.
      * @param A    The first vertex of the edge.
      * @param B    The second vertex of the edge.
      * @param C The opposing vertex of the triangle containing the edge running from
      * A to B. As such the C-vertex defines the plane with respect to which we want
      * to test p.
      *
      * @return     The signed distance of p to the edge-face voronoi plane.
      */
      template< typename V >
      inline typename V::value_type signed_distance_to_edge_face_voronoi_plane( 
        V const & p
        , V const & A
        , V const & B
        , V const & C
        )
      {
        using std::fabs;

        typedef typename V::value_traits    value_traits;
        typedef typename V::value_type      T;

        V m      = cross( A-C, B-C );

        assert( dot( m, m ) > value_traits::zero() || !"signed_distance_to_edge_face_voronoi_plane(): Degenerate triangle encountered");

        V l      = cross( B-A, m );
        V n      = unit( l );

        T sign_p = dot( n, p-B );
        T sign_C = dot( n, C-B );
        T abs_p  = fabs( sign_p );

        assert( is_number( sign_p ) || !"signed_distance_to_edge_face_voronoi_plane(): Not a Number encountered");
        assert( is_number( sign_C ) || !"signed_distance_to_edge_face_voronoi_plane(): Not a Number encountered");
        assert( is_number( abs_p )  || !"signed_distance_to_edge_face_voronoi_plane(): Not a Number encountered");

        bool in_front = ( (sign_p*sign_C) <= value_traits::zero() );

        return in_front ? abs_p : -abs_p;
      }

    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_EDGE_FACE_VORONOI_PLANE_H
#endif
