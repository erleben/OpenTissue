#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_ELLIPSOID_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_ELLIPSOID_H
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
    /**
    * An Ellipsoid.
    */
    template<typename math_types>
    class Ellipsoid
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      V m_scale;   ///< The scaling along the axes, which is applied to the unit-sphere to achieve the ellipsoid. The values are equivalent to the length of the ellipsoid axes.

    public:

      /**
      * Get Ellipsoid Scale.
      *
      * @return    A reference to a vector containing the scale values of the ellipse.
      */
      V const & scale() const { return this->m_scale; }
      V       & scale()       { return this->m_scale; }

    public:

      Ellipsoid()
        : m_scale( value_traits::one(), value_traits::one(), value_traits::one() )
      {}

    public:

      V operator()(V const & v) const
      {
        using std::sqrt;

        assert( this->m_scale(0) >= value_traits::zero() || !"Ellipsoid::operator(): Negative scale encountered");
        assert( this->m_scale(1) >= value_traits::zero() || !"Ellipsoid::operator(): Negative scale encountered");
        assert( this->m_scale(2) >= value_traits::zero() || !"Ellipsoid::operator(): Negative scale encountered");
        //
        // An ellipsoid, E, is simply a scaled unit ball, B, and a scale is a linear
        // transformation, T. We can write it in a general way as
        //
        //  E = T(B)
        //
        // That means we can create a support function, S, of an ellipsoid from
        // that of a unit sphere ball by
        //
        //  S_E(v) = S_{T(B)}(v)
        //
        // Further we know that for any affine transformation, T(v) = R v + t, we have
        //
        //   S_{T(B)}(v)  =   T(  S_B( R^T v )  )
        //
        // In our particular case R = R^T = D, where D = diag(s_0,s_1,s_3) and t=0. Here
        // the s_i's are the axes scales repsectively. Putting it all together we have
        //
        //   S_{E}(v)  =   D(  S_B( D v )  )
        //
        // This is the formula implemented by this functor.
        //
        T vv = dot(v,v);
        assert( is_number(vv) || !"Ellipsoid::operator(): NaN encountered");

        if (vv > value_traits::zero() )
        {
          V const w   = V( 
                           v(0) * (this->m_scale(0))
                         , v(1) * (this->m_scale(1))
                         , v(2) * (this->m_scale(2))
                         );
          T const ww  = dot(w,w);
          assert( is_number(ww) || !"Ellipsoid::operator(): NaN encountered");

          T const tmp = value_traits::one() / sqrt(ww);
          assert( is_number(tmp) || !"Ellipsoid::operator(): NaN encountered");

          V const s   = V( 
                           w(0) * (tmp * (this->m_scale(0)))
                         , w(1) * (tmp * (this->m_scale(1)))
                         , w(2) * (tmp * (this->m_scale(2))) 
                         );
          assert( is_number( s(0) ) || !"Ellipsoid::operator(): NaN encountered");
          assert( is_number( s(1) ) || !"Ellipsoid::operator(): NaN encountered");
          assert( is_number( s(2) ) || !"Ellipsoid::operator(): NaN encountered");

          return s;
        }
        return V( this->m_scale(0), value_traits::zero(), value_traits::zero() );
      }

    };

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_ELLIPSOID_H
#endif
