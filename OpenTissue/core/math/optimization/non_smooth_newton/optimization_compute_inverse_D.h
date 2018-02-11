#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_INVERSE_D_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_INVERSE_D_H
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
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {
      namespace detail
      {

        /**
        * Compute D^{-1}.
        *
        * @param D  This argument holds a matrix with ones along
        *           the diagonal and non-zero values in the strict
        *           lower triangular part. The non-zero pattern is
        *           such that the inverse matrix is given by negating
        *           the strictly lower part of the matrix. Upon return
        *           the argument will hold the inverted matrix.
        *           Each row must only contain exactly one non-zero
        *           value besides the one stored in the diagonal.
        *
        */
        template<typename T>
        inline void compute_inverse_D(  ublas::compressed_matrix<T> & D )
        {
          using std::min;

          typedef OpenTissue::math::ValueTraits<T>  value_traits;

          if(D.size1()>0 && D.size2()>0)
          {
            assert(D.size1() ==  D.size2()|| !"compute_inverse_D(): incompatible dimensions");

            size_t const row_end = D.filled1() - 1;

            for (size_t row = 0; row < row_end; ++ row)
            {
              size_t const begin = D.index1_data()[row];
              size_t const end  = D.index1_data()[row + 1];

              for (size_t j = begin; j < end; ++ j)
              {
                size_t const column = D.index2_data()[j];
                if( row > column)
                {
                  D.value_data()[j] = - D.value_data()[j];
                }
                assert( (row!=column) || ((row == column) && (D.value_data()[j] == value_traits::one() )) || !"compute_inverse_D(): Invalid zero pattern detected");
                assert( (row>=column) || ((row < column)  && (D.value_data()[j] == value_traits::zero() )) || !"compute_inverse_D(): Invalid zero pattern detected");
              }
            }
          }
        }

      } // namespace detail
    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_INVERSE_D_H
#endif
