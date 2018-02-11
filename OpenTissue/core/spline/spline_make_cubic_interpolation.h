#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_MAKE_CUBIC_INTERPOLATION_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_MAKE_CUBIC_INTERPOLATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <OpenTissue/core/spline/spline_nubs.h>
#include <OpenTissue/core/spline/spline_solve_tridiagonal.h>

#include <stdexcept>
#include <cassert>

namespace OpenTissue
{
  namespace spline
  {
    /**
    * Make Interpolating Spline of Cubic Order.
    * Given a set of data points the interpolating B-spline is created and returned.
    *
    * @param U            The knot vector to be used when interpolation the data points.
    * @param X            The data points the cubic B-spline must run through
    *                     In other terms the key-positions of the B-spline.
    */
    template<typename knot_container, typename point_container>
    inline NUBSpline<knot_container, point_container>
    make_cubic_interpolation(
                         knot_container const & U
                       , point_container const & X
                       )
    {
      typedef typename point_container::value_type       V;
      typedef typename knot_container::value_type        T;
      typedef          OpenTissue::math::ValueTraits<T>  value_traits;

      int const k = 4; // The order of the interpolation spline

      //--- Variables: From theory we know:
      //---
      //---   |P| = n + 1         i in (0,1,2,3,...,n-1,n)
      //---   |X| = n - 1         i in (0,1,2,3,...,n-2)
      //---   |U| = |P| + k
      //---   |U| = n + 1 + k     i in (0,1,2,3,...,n+k-1,n+k)
      //---
      size_t const n   = X.size() + 1u;
      size_t const dim = X[0].size();

      if( dim <= 0u )
        throw std::invalid_argument("can not make a non-positive dimensional spline");

      if( U.size() != (n + k + 1u) )
        throw std::invalid_argument("Mismatch between dimension of U and X");

      point_container P;
      
      //--- Initialize controlpoint container.
      P.resize(n-1u);
      size_t i;
      for(i=0u;i< n-1u;++i)
      {
        P[i].resize(dim);
      }
      
      //--- Create the tridiagonal set of equations.
      //--- We have:
      //---
      //---  M P = X
      //---
      //--- Where M is a tridiagonal matrix, X is the
      //--- column of all data points and P is a column
      //--- of all control points in the range 1,...,n-1.
      //---
      //--- It is implicitly assumed that we have a fourth
      //--- order spline (i.e. k=4).
      //---
      //--- We start with the alpha, beta and gamma functions.
      //--- See PBA pp 744 for details.
      V a(n - 1u);
      V b(n - 1u);
      V c(n - 1u);

      a[0]   = value_traits::zero(); b[0]   = value_traits::one(); c[0]   = value_traits::zero();
      a[n-2] = value_traits::zero(); b[n-2] = value_traits::one(); c[n-2] = value_traits::zero();

      for(i=1u;i<n-2u;++i)
      {
        T const & t1 = U[i+1u];
        T const & t2 = U[i+2u];
        T const & t3 = U[i+3u];
        T const & t4 = U[i+4u];
        T const & t5 = U[i+5u];

        assert( t5 >= t4 || !"make_cubic_interpolation(): invalid knot vector");
        assert( t4 >= t3 || !"make_cubic_interpolation(): invalid knot vector");
        assert( t3 >= t2 || !"make_cubic_interpolation(): invalid knot vector");
        assert( t2 >= t1 || !"make_cubic_interpolation(): invalid knot vector");

        a[i] = (t4-t3)*(t4-t3)/(t4-t1);
        b[i] = (t3-t1)*(t4-t3)/(t4-t1) + (t5-t3)*(t3-t2)/(t5-t2);
        c[i] = (t3-t2)*(t3-t2)/(t5-t2);
        a[i] /= (t4-t2);
        b[i] /= (t4-t2);
        c[i] /= (t4-t2);

        assert( is_number( a[i] ) || !"make_cubic_interpolation(): not a number encountered");
        assert( is_number( b[i] ) || !"make_cubic_interpolation(): not a number encountered");
        assert( is_number( c[i] ) || !"make_cubic_interpolation(): not a number encountered");
      }

      //--- Solve for the controlpoints.
      detail::solve_tridiagonal(a,b,c,P,X,n-1u);

      //--- Insert X[0] as the first controlpoint.
      P.insert(P.begin(), 1, X[0]);  

      //--- Insert X[n-2] as the last controlpoint.
      P.push_back(X[n-2u]);

      return NUBSpline<knot_container, point_container>(k,U,P);
    }
  
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_MAKE_CUBIC_INTERPOLATION_H
#endif
