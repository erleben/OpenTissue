#ifndef OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_TRIANGLE_H
#define OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_TRIANGLE_H
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
      * Signed Distance between a point and a Triangle.
      *
      * @param p     A point which we want to know whether it lies on the other
      *              side of the triangle that the point q.
      * @param A     The first vertex of the triangle.
      * @param B     The second vertex of the triangle.
      * @param C     The third vertex of the triangle.
      * @param q     A point that is known to lie on the back-side of the triangle
      *
      *@return       The signed distance of p to the triangle given by vertices A, B, and C. The
      *              point q is used to specify the back side half-plane of the triange.
      */
      template< typename V >
      inline typename V::value_type signed_distance_to_triangle( 
        V const & p
        , V const & A
        , V const & B
        , V const & C
        , V const & q
        )
      {
        using std::fabs;

        typedef typename V::value_traits    value_traits;
        typedef typename V::value_type      T;

        V m = cross( A-B, C-B );

        assert( dot( m, m ) > value_traits::zero() || !"signed_distance_to_triangle(): Degenerate triangle encountered");

        V n = unit( m );

        T sign_p = dot( n, p-B );
        T sign_q = dot( n, q-B );
        T abs_p  = fabs( sign_p );

        assert( sign_q < value_traits::zero() || sign_q > value_traits::zero() || !"signed_distance_to_triangle(): q was in plane, can  not be used to determine sign");
        
        assert( is_number( sign_p ) || !"signed_distance_to_triangle(): Not a Number encountered");
        assert( is_number( sign_q ) || !"signed_distance_to_triangle(): Not a Number encountered");
        assert( is_number( abs_p )  || !"signed_distance_to_triangle(): Not a Number encountered");

        bool in_front = ( (sign_p*sign_q) <= value_traits::zero() );

        return  in_front ? abs_p : - abs_p;
      }

    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SIGNED_DISTANCE_TO_TRIANGLE_H
#endif
