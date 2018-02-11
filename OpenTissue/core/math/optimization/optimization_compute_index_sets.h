#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_INDEX_SETS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_INDEX_SETS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_constants.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {
        /**
        * Compute index sets.
        *
        * @param y             This vector contains the value of the function y = f(x).
        * @param x             The current value of the x-vector.
        * @param l             A functor used to retrieve information about the lower bound function.
        * @param u             A functor used to retrieve information about the upper bound function.
        * @param bitmask       A Bitmask, Upon return this vector flags all variables according to their set memberships.
        *                      The i'th value is equal to ``in active'' if and only if  y(i) \leq (x(i) - lo(i)) &&  y(i) \geq (x(i) - hi(i))
        *                      The i'th value is equal to ``in upper'' if and only if  y(i) < (x(i) - hi(i))
        *                      The i'th value is equal to ``in lower'' if and only if  y(i) > (x(i) - lo(i))       
        * @param cnt_active    Upon return this argument holds the total number of variables in the set of active constraints.
        * @param cnt_inactive  Upon return this argument holds the total number of variables in the union of the set of lower and the set of upper constraints.
        */
        template < typename T, typename bound_function_type>
        inline void compute_index_sets(
            ublas::vector<T> const & y
          , ublas::vector<T> const & x
          , bound_function_type const & l
          , bound_function_type const & u
          , ublas::vector<size_t> & bitmask
          , size_t  & cnt_active
          , size_t  & cnt_inactive
          )
        {
          using std::min;
          using std::max;

          typedef OpenTissue::math::ValueTraits<T>  value_traits;

          size_t n = x.size();
          bitmask.resize(n);

          cnt_active   = 0;
          cnt_inactive = 0;
          size_t cnt_lower = 0;
          size_t cnt_upper = 0;

          for (size_t i = 0; i < n; ++i)
          {
            if(y(i) > (x(i) - l(x,i)))
            {
              bitmask(i) = IN_LOWER;
              ++cnt_lower;
              ++cnt_inactive;
            }
            else if(y(i) < (x(i) - u(x,i)))
            {
              bitmask(i) = IN_UPPER;
              ++cnt_upper;
              ++cnt_inactive;
            }
            else
            {
              // y(i) <= (x(i) - lo(i)) &&  y(i) >= (x(i) - hi(i))
              bitmask(i) = IN_ACTIVE;
              ++cnt_active;
            }
          }
          assert( (cnt_active + cnt_inactive) == n        || !"compute_index_sets(): index sets were inconsistent");
          assert( (cnt_lower + cnt_upper) == cnt_inactive || !"compute_index_sets(): index sets were inconsistent");
        }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_INDEX_SETS_H
#endif
