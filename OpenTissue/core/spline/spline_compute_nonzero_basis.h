#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_NONZERO_BASIS_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_NONZERO_BASIS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>

#include <OpenTissue/core/spline/spline_compute_knot_span.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * Compute nonzero basisfunctions.
      * Uses a triangular scheme to compute only nonzero k k-order basisfunctions.
      *
      * Disregarding the dynamic memory allocation in this method it is
      * infact the optimal implementation. For details look at pp.68-70 in
      * "The Nurbs Book" by L. Piegl and W. Tiller.
      *
      * @param u  Paramter value.
      * @param k  Order of the basisfunction.
      * @param U  Knotvector.
      * @param N  A reference to a vector, which upon return will
      *           contain the values of the basisfunctions.
      */
      template<typename vector_type, typename knot_container>
      inline void compute_nonzero_basis(
        typename knot_container::value_type const & u
        , int const k
        , knot_container const & U
        , vector_type & N
        )
      {
        using std::fabs;

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

        int const i = compute_knot_span(u, k, U);

        vector_type left(k+1);
        vector_type right(k+1);

        N.resize(k, false);

        N(0) = value_traits::one();

        for(int j=1; j<k; ++j)
        {
          left(j)  = u - U[i+1-j];
          right(j) = U[i+j] - u;

          T saved = value_traits::zero();

          for(int r=0; r<j; ++r)
          {

            T tmp       = value_traits::zero();
            T nominator = right(r+1) + left(j-r);

            if(fabs( nominator ) > value_traits::zero() )
            {
              tmp   = N(r)/nominator;
              N(r)  = saved + right(r+1)*tmp;
              saved = left(j-r)*tmp;
            }
            else
            {
              N(r)  = saved;
              saved = value_traits::zero();
            }

          }
          N(j) = saved;
        }
        //--- Now N contains the values N(i-k+1,k),..,N(i,k) in
        //--- that order.
      }

    } // namespace detail
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_NONZERO_BASIS_H
#endif
