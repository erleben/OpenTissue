#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_STATIONARY_POINT_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_STATIONARY_POINT_H
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
#include <OpenTissue/core/math/math_is_number.h>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * Stationary Point Test Function
      *
      * @param gradient     The current gradient value. If the gradient is
      *                     zero then we have a stationary point. In this test
      *                     we simply test if the length of the gradient is close
      *                     enough to zero within a given user specified tolerance.
      * @param tolerance    This argument holds the value used in the absolute stopping
      *                     criteria. Setting the value to zero will make the test
      *                     in-effective.
      * @param length       Upon return this argument holds the value of the length
      *                     of the gradient.
      *
      * @return             If the 2 norm of the gradient is less than the given
      *                     threshold then the return value is true otherwise
      *                     it is false.
      */
      template <         typename T      >
      inline bool stationary_point( ublas::vector<T> const & gradient, T const & tolerance, T & length )  
      {
        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        assert( is_number( tolerance )          || !"stationary_point(): internal error, NAN is encountered?");
        assert( tolerance>=value_traits::zero() || !"stationary_point(): internal error, NAN is encountered?");

        length = ublas::norm_2( gradient) ;

        assert( is_number( length ) || !"stationary_point(): internal error, NAN is encountered?");

        if( length <= tolerance)
          return true;
        return false;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_STATIONARY_POINT_H
#endif
