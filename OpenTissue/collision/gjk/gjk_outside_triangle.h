#ifndef OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_TRIANGLE_H
#define OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_TRIANGLE_H
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
      * Test if point is outside a Triangle.
      *
      * @param p     A point which we want to know whether it lies on the other
      *              side of the triangle that the point q.
      * @param A     The first vertex of the triangle.
      * @param B     The second vertex of the triangle.
      * @param C     The third vertex of the triangle.
      * @param q     A point that is known to lie on the back-side of the triangle
      *
      *@return       If p is outside or on the face plane then the return value is true otherwise it is false.
      */
      template< typename V >
      inline bool outside_triangle( 
        V const & p
        , V const & A
        , V const & B
        , V const & C
        , V const & q
        )
      {
        typedef typename V::value_traits    value_traits;
        typedef typename V::value_type      T;

        V const n = cross( A-B, C-B );

        assert( dot( n, n ) > value_traits::zero() || !"outside_triangle(): Degenerate triangle encountered");

        T const sign_p = dot( n, p-B );
        T const sign_q = dot( n, q-B );

        assert( is_number( sign_p ) || !"outside_triangle(): Not a Number encountered");
        assert( is_number( sign_q ) || !"outside_triangle(): Not a Number encountered");

        assert( sign_q < value_traits::zero() || sign_q > value_traits::zero() || !"outside_triangle(): q was in plane, can  not be used to determine sign");        

        return (sign_p*sign_q) <= value_traits::zero();
      }

    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_OUTSIDE_TRIANGLE_H
#endif
