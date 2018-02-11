#ifndef OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_VECTOR_H
#define OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_VECTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Interval Vector - Vector Dot product.
      * This templat function implements the operation
      *
      *   [[x1,x2],[y1,y2],[z1,z2]] * [x,y,z] = [x1,x2]*x+[y1,y2]*y+[z1,z2]*z.
      *
      * @param vi    The interval vector.
      * @param vr    The vector
      *
      * @return      The resulting interval type.
      */
      template<typename interval_type>
      inline interval_type operator*( OpenTissue::math::Vector3<interval_type > const & vi, OpenTissue::math::Vector3<typename interval_type::base_type > const & vr)
      {
        typedef typename interval_type::base_type     T;
        //typedef typename interval_type::value_traits  value_traits;  // Argh!!! boost interval type do not have this:-(
        typedef typename math::ValueTraits<T>         value_traits;

        T x = vr[0];
        T tmp_lower = (x< value_traits::zero() ? vi[0].upper() : vi[0].lower() ) * x;
        T tmp_upper = (x< value_traits::zero() ? vi[0].lower() : vi[0].upper() ) * x;

        T y = vr[1];
        tmp_lower  += (y< value_traits::zero() ? vi[1].upper() : vi[1].lower() ) * y;
        tmp_upper  += (y< value_traits::zero() ? vi[1].lower() : vi[1].upper() ) * y;

        T z = vr[2];
        tmp_lower  += (z< value_traits::zero() ? vi[2].upper() : vi[2].lower() ) * z;
        tmp_upper  += (z< value_traits::zero() ? vi[2].lower() : vi[2].upper() ) * z;

        return interval_type(tmp_lower, tmp_upper);
      }

    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_VECTOR_H

#endif 
