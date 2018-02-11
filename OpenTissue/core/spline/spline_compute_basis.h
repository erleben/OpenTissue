#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_BASIS_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_BASIS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * The i'th Basis Blending Function.
      * This method implements the de Cox's recursive definition of a
      * B-spline basis function.
      *
      * [If time permits implement optimized version NURBS page 74]
      *
      * @param i  The index of the basis function.
      * @param k  The order of the spline (and not the degree, which are k-1).
      * @param u  The parameter value at which the i'th basis function should
      *           be evaluated at.
      * @param T  Knotvector.
      *
      * @return   The value of the i'th k'th order basis function at the
      *           parameter value u.
      */
      template<typename knot_container>
      inline typename knot_container::value_type compute_basis(
        int const & i
        , int const & k
        , typename knot_container::value_type const & u
        , knot_container const & U
        )
      {
        using std::fabs;

        typedef typename knot_container::value_type       T;
        typedef          OpenTissue::math::ValueTraits<T> value_traits;

        if(k<1)
          throw std::invalid_argument("order must be greater than zero");

        if(i<0)
          throw std::invalid_argument("basis function index must be non-negative");

        if( 0 >= (i + k - U.size())   )
          throw std::invalid_argument("No such basis function can exist");

        if(k==1)
        {
          int const m = U.size() - 1;
          int const n = m - k;

          // special case; test if we are touching end-point of spline
          if(u==U[i+1] && U[i+1]==U[n+k])
            return value_traits::one();

          if((U[i] <= u) && (u < U[i+1]))
          {
            return value_traits::one();
          }
          else
          {
            return value_traits::zero();
          }
        }
        else
        {
          T a = U[i+k-1] - U[i];
          if( fabs(a) > value_traits::zero() )
          {
            T tmp = compute_basis(i, k-1, u, U);
            // a = ( fabs(tmp)> value_traits::zero() ) ? ((u-U[i])*tmp)/a  :  value_traits::zero();
            a = ((u-U[i])*tmp)/a;
          }

          T b = U[i+k]-U[i+1];
          if( fabs(b) > value_traits::zero() )
          {
            T tmp = compute_basis(i+1,k-1,u, U);
            // b = ( fabs(tmp)> value_traits::zero() ) ? ((U[i+k]-u)*tmp)/b  : value_traits::zero();
            b = ((U[i+k]-u)*tmp)/b;
          }

          return a + b;
        }
      }

    } // namespace detail
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_BASIS_H
#endif
