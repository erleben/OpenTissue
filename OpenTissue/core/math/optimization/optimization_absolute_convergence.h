#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_ABSOLUTE_CONVERGENCE_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_ABSOLUTE_CONVERGENCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <cmath>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * Absolute Convergence Test Function.
      *
      * @param f            The function value at the current iterate.
      * @param tolerance    This argument holds the value used in the absolute
      *                     stopping criteria.
      *                     Setting the value to zero will make the test in-effective.
      * @return             If the absolute convergence test passes then the return
      *                     value is true otherwise it is false.
      */
      template < typename T >
      inline bool absolute_convergence(
          T const & f
        , T const & tolerance
        )
      {
        using std::fabs;

        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        assert( tolerance >= value_traits::zero() || !"absolute_convergence(): tolerance must be non-negative");
        assert( is_number( tolerance )            || !"absolute_convergence(): internal error, NAN is encountered?");
        assert( is_number( f )                    || !"absolute_convergence(): internal error, NAN is encountered?");
        
        if(f <= tolerance)
          return true;

        return false;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_ABSOLUTE_CONVERGENCE_H
#endif
