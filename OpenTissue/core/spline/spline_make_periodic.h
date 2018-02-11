#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_MAKE_PERIODIC_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_MAKE_PERIODIC_H
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

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * Convert the knot-vector and controlpoints to closed periodic.
      * Afterwards they can be used to draw a closed B-spline curve.
      * See pp 200 Applied Geometry for Computer Graphics and CAD 2nd Edition. 
      *
      * @param k  Order.
      * @param U  Knot-vector. Must contain n+2 knots. Will be updated to contain:
      *           (n+2*d+2) knots where d=k-1.       
      * @param P  A controlpoint container. Must contain n+1 controlpoints.
      */
      template<typename knot_container, typename point_container>
      inline void compute_closed_periodic(int const k, knot_container & U, point_container & P)
      {
        typedef typename knot_container::value_type T;

        int const n = P.size()-1;
        int const d = k-1;
        int const knot_num = n+2*d+2;

        //--- Add the new knot-values and controlpoints.
        for (int i = 0; i < d; ++i)
        {
          P.push_back(P[i]);
        }

        int j = 1;
        for (int i = n+2; i < knot_num; ++i)
        {
          T val = U[i-1] + (U[j] - U[j-1]);
          U.push_back(val);
          ++j;
        }    

      }
    } // namespace detail

    /**
    * Make Closed Periodic Spline.
    * This factory function will make a closed periodic spline
    * from a non-closed one.
    *
    * @param spline     The non-periodic non-closed spline
    *
    * @return           A new spline corresponding to the non-closed one expcept
    *                   that it has been turned into a closed periodic spline.
    */
    template<typename spline_type>
    inline spline_type make_periodic(spline_type const & spline)
    {
      spline_type S;
      S = spline;
      detail::compute_closed_periodic( S.get_order(), S.get_knot_container(), S.get_control_container() );
      return S;
    }

  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_MAKE_PERIODIC_H
#endif
