#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_STAGNATION_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_STAGNATION_H
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
#include <OpenTissue/core/math/big/big_types.h>

#include <cmath>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * Stagnation Test Function.
      *
      * @param x_old                 The previous iterate value.
      * @param x                     The next iterate value.
      * @param tolerance             This argument holds the value used in the stagnation test. It is
      *                              an upper bound of the infinity-norm of the difference in the x-solution
      *                              between two iterations.  Setting the value to zero will make the test in-effective.
      * @return                      If stagnation test passes then the return value is true otherwise it is false.
      */
      template < typename T >
      inline bool stagnation(
          ublas::vector<T> const & x_old
        , ublas::vector<T> const & x
        , T const & tolerance
        )
      {
        using std::fabs;
        using std::max;


        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        assert( tolerance >= value_traits::zero() || !"stagnation(): tolerance must be non-negative");
        assert( is_number( tolerance )            || !"stagnation(): internal error, NAN is encountered?");

        size_t const m = x.size();

        assert( m>0                      || !"stagnation(): zero size of x");
        assert( x.size() == x_old.size() || !"stagnation(): incompatible dimensions of x and x_old");

        T max_dx = value_traits::zero();

        for (size_t i = 0; i < m; ++ i)
          max_dx = max( max_dx, fabs( x(i) - x_old(i) ) );

        assert( is_number( max_dx ) || !"stagnation(): internal error, NAN is encountered?");

        if(max_dx <= tolerance)
          return true;
        return false;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_STAGNATION_H
#endif
