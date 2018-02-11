#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_POINT_ON_SPLINE_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_POINT_ON_SPLINE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/spline/spline_compute_knot_span.h>
#include <OpenTissue/core/spline/spline_compute_nonzero_basis.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * Get Point On Spline.
      * Computes af point on a B-spline. Disregarding the dynamic
      * memory allocation in this method it is infact the optimal implementation.
      * For details look at pp.68-70 in "The Nurbs Book" by L. Piegl and W. Tiller.
      *
      * @param u  Parameter value.
      * @param k  Order of basisfunction.
      * @param U  Knot-vector.
      * @param P  A controlpoint container.
      * @param p  A reference to a vector, which upon return will
      *           contain the computed point on the spline.
      */
      template<typename knot_container, typename point_container>
      inline void compute_point_on_spline(
        typename knot_container::value_type const  & u
        , int const k
        , knot_container const & U
        , point_container const & P
        , typename point_container::value_type & p
        )
      {
        typedef typename point_container::value_type vector_type;

        if(k<1)
          throw std::invalid_argument("order must be greater than zero");

        if( U.size() != (P.size() + k) )
          throw std::invalid_argument("invalid spline specification");

        vector_type N;
        //--- Get the k nonzero basisfuntions.
        detail::compute_nonzero_basis(u, k, U, N);
        //--- Now N contains the values N(i-k+1,k),..,N(i,k) in
        //--- that order.
        int const i = detail::compute_knot_span(u, k, U);

        p.clear();
        for(int r=0; r < k; ++r)
        {
          int j = i-k+1+r;
          p += P[j] * N(r);
        }
      }

    } // namespace detail


    /**
     * Compute Point on A Spline.
     *
     * @param u        The spline parameter value at which the point should
     *                 be computed.
     * @param spline   The spline from which a point should be computed.
     * @param p        Upon return this argument holds the value of the
     *                 point at the spline paramter value u.
     */
    template<typename spline_type>
    inline void compute_point_on_spline(
      typename spline_type::knot_container::value_type const  & u
      , spline_type const & spline
      , typename spline_type::point_container::value_type & p
      )
    {
      detail::compute_point_on_spline(
        u
        , spline.get_order()
        , spline.get_knot_container()
        , spline.get_control_container()
        , p
        );
    }

  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_POINT_ON_SPLINE_H
#endif
