#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_INITIALIZE_M_TABLE_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_INITIALIZE_M_TABLE_H
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
      * Initialize the M table used when computing the derivatives.
      * The M table is a k*k matrix containing values for up to 
      * k k-order nonzero basisfunctions and their corresponding
      * knot-differences.
      *
      * @param i  The index of the basis function.
      * @param u  The parameter value at which the i'th basis function should
      *           be evaluated at.
      * @param k  The order of the spline (and not the degree, which are k-1).
      * @param U  A reference to the knotvector. It will have n+k+1 knot values.
      * @param M  A reference to the M table that will be initialized when the function returns.
      */
      template<typename matrix_type, typename knot_container>
      inline void initialize_m_table(
        int const & i
        , typename knot_container::value_type const & u
        , int const & k
        , knot_container const & U
        , matrix_type & M
        )
      {
        typedef typename knot_container::value_type        T;
        typedef          OpenTissue::math::ValueTraits<T>  value_traits;

        if(k<1)
          throw std::invalid_argument( "order must be greater than zero" );

        if( U.size() < 2u*k)
          throw std::invalid_argument( "knot vector did not have enough entries" );

        if( u < U[0] )
          throw std::invalid_argument( "u-parameter was out of lower bound" );

        if( u > U[ U.size() - 1 ] )
          throw std::invalid_argument( "u-parameter was out of upper bound" );

        if(i<0)
          throw std::invalid_argument( "basis function index must be non-negative" );

        knot_container left(k+1);
        knot_container right(k+1);

        M.resize(k,k,false);
        M(0,0) = value_traits::one();

        for (int j=1; j<k; ++j)
        {
          left[j]  = u-U[i+1-j];
          right[j] = U[i+j]-u;

          T saved = value_traits::zero();

          for(int r = 0; r<j; ++r)
          {
            /* Lower triangle (knot differences) */
            M(j,r) = right[r+1] + left[j-r];
            T temp = M(r,j-1)/M(j,r);

            /* Upper triangle (basisfunctions values) */   
            M(r,j) = saved+right[r+1]*temp;
            saved = left[j-r]*temp;
          }
          M(j,j) = saved;
        }
      }

    } // namespace detail
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_INITIALIZE_M_TABLE_H
#endif
