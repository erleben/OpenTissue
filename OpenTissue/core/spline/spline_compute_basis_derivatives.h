#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_BASIS_DERIVATIVES_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_BASIS_DERIVATIVES_H
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
#include <OpenTissue/core/spline/spline_initialize_m_table.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * Computation of derivatives of the basisfunctions.
      * This method efficiently computes all derivatives from 0 upto J'th order
      * of the basisfunctions at the parameter value u. Note that the 0'th order derivative
      * is simply the basisfunctions themselves.
      *
      * The method can for instance be used to retrieve velocity and acceleration
      * information about an object moving along the spline.
      *
      * @param u      The parameter value at which the derivatives should be computed.
      * @param J      The highest order derivative, which should be computed, should be
      *               less than the order of the spline since all other higher order
      *               derivatives is trivially known to be zero.
      * @param k      Order of basisfunction.
      * @param U      The knot-vector.
      * @param dQ     dQ is a (J+1,k) matrix which upon return will contain the computed derivatives.
      */
      template<typename knot_container, typename matrix_type>
      inline void compute_basis_derivatives(
        typename knot_container::value_type const & u
        , int J
        , int const & k
        , knot_container const & U
        , matrix_type & dQ
        )
      {
        using std::min;
        using std::fabs;

        typedef typename knot_container::value_type       T;
        typedef          OpenTissue::math::ValueTraits<T> value_traits;

        if(J<1)
          throw std::invalid_argument( "The derivative must be at least of first-order" );

        if(k<1)
          throw std::invalid_argument( "order must be greater than zero" );

        if( U.size() < 2u*k)
          throw std::invalid_argument( "knot vector did not have enough entries" );

        if( u < U[0] )
          throw std::invalid_argument( "u-parameter was out of lower bound" );

        if( u > U[ U.size() - 1 ] )
          throw std::invalid_argument( "u-parameter was out of upper bound" );


        int const i = compute_knot_span(u, k, U);

        //--- Initialize M.
        matrix_type M;
        initialize_m_table(i,u,k,U,M);

        //--- Resize the dQ to contain the derivatives.
        dQ.resize(J+1,k,false);

        //--- Safety control, all higher order derivatives with J>=K is
        //--- trivially known to be zero.
        J = min(J,k-1);

        //--- Trivially we already know the 0'th derivatives
        //--- of the basis functions they are simply the basis
        //--- functions themself.
        for(int r=0; r<k; ++r)
        {
          dQ(0,r) = M(r,k-1);
        }

        //--- Now we must compute the remaining higher order derivatives.
        //--- We will need some working storage to hold the values of the a-coefficient.
        matrix_type a(2,k);

        //--- Out loop is over function indices
        for(int r=0; r<=k-1; ++r)
        {
          int s1 = 0;
          int s2 = 1;
          a(0,0) = value_traits::one();

          //--- Inner loop is over the derivatives
          for(int j=1;j<=J;++j)
          {
            T djN = value_traits::zero();

            //--- The case of a(j,0)
            if(r >= j)
            {
              if( fabs( M(k-j,r-j) ) > value_traits::zero() )
                a(s2,0) = a(s1,0) / M(k-j,r-j);
              else
                a(s2,0) = value_traits::zero();

              djN +=  a(s2,0) * M(r-j,k-j-1);
            }

            //--- The case of a(j,1) to a(j,j-1)
            int pMin,pMax;

            if((r + 1) >= j)
            {//--- determine lowest p for which we have a non-zero basis function
              pMin = 1;
            }
            else
            {
              pMin = j-r;
            }

            if(r <= (k-j))
            {//--- determine highest p for which we have a non-zero basis function
              pMax = j - 1;
            }
            else
            {
              pMax = k - 1 - r;
            }

            for(int p=pMin; p<=pMax; ++p)
            {
              if( fabs( M(k-j,r-j+p) ) > value_traits::zero() )
                a(s2,p) = (a(s1,p)-a(s1,p-1)) / M(k-j,r-j+p);
              else
                a(s2,p) = value_traits::zero();

              djN += a(s2,p)*M(r-j+p,k-j-1);
            }

            //--- The case of a(j,j)
            if(r <= (k-1-j))
            {
              if( fabs( M(k-j,r) ) > value_traits::zero() )
                a(s2,j) = - a(s1,j-1) / M(k-j,r);
              else
                a(s2,j) = value_traits::zero();

              djN +=  a(s2,j) * M(r,k-j-1);
            }

            dQ(j,r) = djN;

            //--- We only need the currently computed row of a-coefficients
            //--- in the next iteration. Swapping indices allows us to use
            //--- row s2 for computing the a-coefficients in the next iteration
            //--- and row s1 will then hold the old values in the next iteration.
            int tmp = s1;
            s1 = s2;
            s2 = tmp;
          }
        }

        //---All that remains is to premultiply by the factor (h-1)!/(h-j-1)!
        T factor = k-1;
        for(int j=1;j<=J;++j)
        {
          for(int r=0;r<k;++r)
          {
            dQ(j,r) = factor * dQ(j,r);
          }
          factor = factor * (k-1-j);
        }
      }

    } // namespace detail
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_BASIS_DERIVATIVES_H
#endif

