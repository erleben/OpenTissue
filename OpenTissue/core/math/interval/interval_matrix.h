#ifndef OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_MATRIX_H
#define OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_MATRIX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/interval_vector.h>
#include <OpenTissue/core/math/math_matrix3x3.h>

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Interval Matrix Vector Multiplication.
      * Interval Matrix vector multiplication uses operator*(interval_vector, vector).
      *
      * This operator is for instance used when we rotate a point with a bounded orientation matrix.
      *
      * @param m   The specified interval matrix.
      * @param v   The specified vector.
      *
      * @return    The resulting interval vector.
      */
      template<typename interval_type>
      inline OpenTissue::math::Vector3< interval_type >  operator*( OpenTissue::math::Matrix3x3< interval_type > const & m, OpenTissue::math::Vector3<typename interval_type::base_type> const & v)
      {
        typedef OpenTissue::math::Vector3< interval_type > interval_vector3_type;
        return interval_vector3_type(
          m.row(0) * v
          , m.row(1) * v
          , m.row(2) * v
          );
      }

    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_MATRIX_H
#endif 
