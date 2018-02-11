#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_NATURAL_MERIT_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_NATURAL_MERIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <cmath>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * Compute value of natural merit function.
      *  
      * @param  F   This argument is expected to hold the vector value of a vector-function.
      *
      * @return     Upon return this function returns the value: theta(x) = F(x)^T F(x)/2.
      */
      template < typename T >
      inline T compute_natural_merit( ublas::vector<T> const & F )  
      {
        using std::min;
        using std::max;

        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        static T const one_half = value_traits::one()/value_traits::two();

        size_t m = F.size();

        if(m==0)
          return value_traits::zero();

        T theta = value_traits::zero();
        for (size_t i = 0; i < m; ++ i)
        {
          T const f_i = F(i);
          theta += f_i*f_i;
        }
        theta *= one_half;
        return theta;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_NATURAL_MERIT_H
#endif
