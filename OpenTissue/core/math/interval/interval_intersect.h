#ifndef OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_INTERSECT_H
#define OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_INTERSECT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/interval_fwd.h>  //--- Needed for forward declaration of interval class
#include <cmath>                                    //--- Needed for std::min and std::max

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Interval Intersection.
      *
      * @param A     The first interval.
      * @param B     The second interval.
      *
      * @return      The resulting interval representing the intersection of the two specified intervals.
      */
      template<typename value_type>
      inline Interval<value_type> intersect( Interval<value_type> const & A, Interval<value_type> const & B)
      {
        using std::min;
        using std::max;

        typedef Interval<value_type>  interval_type;

        //--- TODO KE : why use register?
        register value_type tmp_lower; // jackj: -O4 probably makes register
        register value_type tmp_upper; // jackj: -O4 probably makes register

        if(empty(A) || empty(B))
          return interval_type::empty();

        if(B.lower()>A.upper() || A.lower()>B.upper())
          return interval_type::empty();

        tmp_lower = max( A.lower(), B.lower() );
        tmp_upper = min( A.upper(), B.upper() );

        return interval_type(tmp_lower, tmp_upper);
      }

    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_INTERSECT_H

#endif 

