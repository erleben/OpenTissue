#ifndef OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_EDGE_FACE_VORONOI_PLANE_H
#define OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_EDGE_FACE_VORONOI_PLANE_H
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
      * Test if Point is outside Edge Face Voronoi Plane.
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
      * @return     If point is outside or on the plane then the return value is true otherwise it is false.
      */
      template< typename V >
      inline bool outside_edge_face_voronoi_plane( 
          V const & p
        , V const & A
        , V const & B
        , V const & C
        )
      {
        typedef typename V::value_traits    value_traits;
        typedef typename V::value_type      T;

        V const m = cross( A-C, B-C );
        assert( dot( m, m ) > value_traits::zero() || !"outside_edge_face_voronoi_plane(): Degenerate triangle encountered");

        V const n      = cross( B-A, m );
        T const sign_p = dot( n, p-B );
        T const sign_C = dot( n, C-B );

        assert( is_number( sign_p ) || !"outside_edge_face_voronoi_plane(): Not a Number encountered");
        assert( is_number( sign_C ) || !"outside_edge_face_voronoi_plane(): Not a Number encountered");

        return (sign_p*sign_C) <= value_traits::zero();
      }

    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_EDGE_FACE_VORONOI_PLANE_H
#endif
