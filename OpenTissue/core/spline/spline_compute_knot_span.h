#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_KNOT_SPAN_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_KNOT_SPAN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * Find Knot Span
      * Based on the algorithm in The NURBS Book pp 68.
      *
      *   U[j] <= u < U[j+1] (*)
      *
      * The knot vector is defined to be
      *
      *   U = [ u0, u1, ..., u_{k-1}, u_k, u_{k+1}, ..., u_{n-1}, u_n, u_{n+1}, ..., u_{n+k} ]
      *
      * Where
      *
      *  u_0 = u_1 = ... = u_{k-1}  and u_{n+1} = u_{n+2} = ... = u_{n+k}
      *
      * @param u  The parameter value.
      * @param k  The order of the spline (and knot the degree, which are k-1).
      * @param U  A reference to the knot vector. It will have n+k+1 knot values.
      * @return   The index value of the lowest knot value in the
      *           knot span, which overlaps with u. In other words
      *           the value of j in the equation (*).
      */
      template <typename knot_container>
      inline int compute_knot_span(
        typename knot_container::value_type const & u
        , int const & k
        , knot_container const & U
        )
      {
        if(k < 1)
          throw std::invalid_argument("order must be greater than zero");

        if(U.size() < 2u*k)
          throw std::invalid_argument("knot vector was too small or order was too high");

        int const m = U.size() - 1;
        int const n = m-k;

        if (u >= U[n+1])
        {
          return n;
        }

        if (u <= U[k-1])
        {
          return k-1;
        }

        int low  = k-1;
        int high = n+1;
        int mid  = (low+high)/2;

        while(u < U[mid] || u >= U[mid+1])
        {
          if (u < U[mid])
          {
            high = mid;
          }
          else
          {
            low = mid;
          }
          mid = (low+high)/2;
        }
        return mid;

      }

    } // namespace detail
  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_KNOT_SPAN_H
#endif
