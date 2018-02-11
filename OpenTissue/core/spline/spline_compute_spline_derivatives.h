#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_SPLINE_DERIVATIVES_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_SPLINE_DERIVATIVES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/spline/spline_compute_basis_derivatives.h>
#include <OpenTissue/core/spline/spline_compute_knot_span.h>

namespace OpenTissue
{
  namespace spline
  {
    /**
    * Computation of derivatives of a B-spline.
    *
    * The method can for instance be used to retrieve velocity and acceleration
    * information about an object moving along the spline.
    *
    * @param s      A reference to a B-spline.
    * @param u      The parameter value at which the derivatives should be computed.
    * @param J      The highest order derivative, which should be computed, should be
    *               less than the order of the spline since all other higher order
    *               derivatives is trivially known to be zero.
    * @param dQ     dQ is a point_container which upon return will contain the computed derivatives
    *               for each component in a the controlpoint.
    */
    template<typename spline_type, typename math_types>
    inline void compute_spline_derivatives(
        spline_type const & s
      , typename spline_type::knot_container::value_type const & u
      , int J
      , typename spline_type::point_container & dQ
      , math_types const & /*math_types_tag*/
      )
    {
      using std::min;

      typedef typename spline_type::knot_container    knot_container;
      typedef typename spline_type::point_container   point_container;
      typedef typename point_container::value_type    vector_type;
      typedef typename math_types::matrix_type        matrix_type;
      
      //--- Get B-spline data
      knot_container  const & U = s.get_knot_container();
      point_container const & P = s.get_control_container();

      int const k   = s.get_order();
      int const dim = s.get_dimension();
      int const i   = detail::compute_knot_span(u, k, U);

      //--- Safety control, all higher order derivatives with J>=K is
      //--- trivially known to be zero.
      J = min(J,k-1);

      //--- Compute the derivatives of the basisfunctions
      matrix_type dQ_basis;

      detail::compute_basis_derivatives(u, J, k, U, dQ_basis);

      //--- Now let us compute the derivatives of the B-spline

      dQ.clear();

      vector_type tmp(dim);
      vector_type curr(dim);

      for(int j=0; j<=J; ++j)
      {
        curr.clear();
        for(int nn=0; nn<=k-1; ++nn)
        {
          tmp = dQ_basis(j,nn) * P[i-k+1+nn];
          curr +=tmp;
        }
        dQ.push_back(curr);
      }
    }

  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_COMPUTE_SPLINE_DERIVATIVES_H
#endif
