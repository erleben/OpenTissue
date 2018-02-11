#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_RELATIVE_CONVERGENCE_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_RELATIVE_CONVERGENCE_H
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
      * Relative Convergence Test Function.
      *
      * @param f_old        The function value at the previous iterate.
      * @param f            The function value at the current iterate.
      * @param tolerance    This argument holds the value used in the relative stopping criteria.
      *                     Setting the value to zero will make the test in-effective.
      * @return             If the relative convergence test passes then the return value is true otherwise it is false.
      */
      template < typename T >
      inline bool relative_convergence(
          T const & f_old
        , T const & f
        , T const & tolerance
        )
      {
        using std::fabs;

        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        assert( tolerance >= value_traits::zero() || !"relative_convergence(): tolerance must be non-negative");
        assert( is_number( tolerance )            || !"relative_convergence(): internal error, NAN is encountered?");
        assert( is_number( f_old )                || !"relative_convergence(): internal error, NAN is encountered?");
        assert( is_number( f )                    || !"relative_convergence(): internal error, NAN is encountered?");


        if(f_old == value_traits::zero())
        {
          // f and f_old are both zero
          if(f == value_traits::zero())
            return true;
          // f_old is zero and f is non-zero, so the test does not really make sense!
          return false;
        }

        T const relative_test = fabs(f - f_old) / fabs(f_old);

        assert( is_number( relative_test ) || !"relative_convergence(): internal error, NAN is encountered?");
        
        if(relative_test <= tolerance)
          return true;

        return false;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_RELATIVE_CONVERGENCE_H
#endif
