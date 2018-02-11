#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_SOLVE_TRIDIAGONAL_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_SOLVE_TRIDIAGONAL_H
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
  namespace spline
  {
    namespace detail
    {
      /**
      * This method solves a tridiagonal linear equation set in m-dimensions.
      *
      *   M P = X
      *
      * where M is a tridiagonal matrix defined as follows:
      *
      * | b[0] c[1]                     |
      * | a[1] b[1] c[2]                |
      * |      a[2]  ..                 |
      * |                 b[n-2] c[n-2] |
      * |                 a[n-2] b[n-1] |
      *
      * @param a-c          The explict basis functions from PBA pp 744.
      * @param P            Upon return P will contains the determined controlpoints.
      * @param X            The datapoints the cubic B-spline must run through.
      * @param n            Index from the knot-vector. Called with n-1
      */
      template<typename vector_type, typename point_container>
      inline void solve_tridiagonal(
        vector_type const & a
        , vector_type const & b
        , vector_type const & c
        , point_container & P
        , point_container const & X
        , int const n
        )
      {
        typedef typename vector_type::value_type T;
        typedef          point_container         matrix_type;   // This is a cheat, we really do not need matrix so this will beautifully

        int i,j;
        int const dim = X[0].size();

        vector_type g(n);

        // allocate space for d matrix!
        matrix_type d;
        d.resize(n);
        for(i=0;i<n;++i)
        {
          d[i].resize(dim);
        }

        //--- Forward elimination sweep (read the datapoints)
        g(0) = c[0]/b[0];
        assert( is_number( g(0) ) || !"solve_tridiagonal(): NaN encountered");

        for(j=0; j<dim; ++j)
        {
          d[0](j) = X[0](j)/b[0];
          assert( is_number( d[0](j) ) || !"solve_tridiagonal(): NaN encountered");
        }

        for(i=1; i<n; ++i)
        {
          T tmp = b[i] - a[i]*g(i-1);
          g(i) = c[i]/tmp;
          assert( is_number( g(i) ) || !"solve_tridiagonal(): NaN encountered");

          for(j=0; j<dim; ++j)
          {
            d[i](j) = (X[i](j) - a[i]*d[i-1](j)) /tmp;
            assert( is_number( d[i](j) ) || !"solve_tridiagonal(): NaN encountered");
          }
        }

        //--- Backward substitution sweep (write the controlpoints)
        for(j=0; j<dim; ++j)
        {
          P[n-1](j) = d[n-1](j);
          assert( is_number( P[n-1](j) ) || !"solve_tridiagonal(): NaN encountered");
        }

        for(i=n-2; i>=0; --i)
        {
          for(j=0; j<dim; ++j)
          {
            P[i](j) = d[i](j) - g(i) * P[i+1](j);
            assert( is_number( P[i](j) ) || !"solve_tridiagonal(): NaN encountered");
          }
        }
      }

    } // namespace detail
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_SOLVE_TRIDIAGONAL_H
#endif
