#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_JACOBIAN_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_JACOBIAN_H
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
      namespace detail
      {

        /**
        * Compute Jacobian.
        *
        * @param A            The coefficient matrix of the linear equation y = A x + b
        * @param l            A functor used to retrieve information about the lower bound function.
        * @param u            A functor used to retrieve information about the upper bound function.
        * @param bitmask      A bitmask. 
        *                     The i'th value is equal to ``in active'' (=4) if and only if  y(i) \leq (x(i) - lo(i)) &&  y(i) \geq (x(i) - hi(i))
        *                     The i'th value is equal to ``in upper'' (=2) if and only if  y(i) < (x(i) - hi(i))
        *                     The i'th value is equal to ``in lower'' (=1) if and only if  y(i) > (x(i) - lo(i))
        * @param J            Upon return this argument holds the value of the Jacobian matrix.
        */
        template<typename T, typename bound_function_type>
        inline void  compute_jacobian( 
          ublas::compressed_matrix<T> const & A
          , bound_function_type const & l
          , bound_function_type const & u
          , ublas::vector<size_t> const & bitmask
          , ublas::compressed_matrix<T>  & J
          )
        {
          using std::min;

          typedef typename bound_function_type::vector_iterator  vector_iterator;
          typedef typename OpenTissue::math::ValueTraits<T>      value_traits;

          static size_t const in_lower  = 1;
          static size_t const in_upper  = 2;
          static size_t const in_active = 4;

          size_t const m = A.size1();
          size_t const n = A.size2();

          J.resize(m,n,false);

          size_t const row_end = A.filled1() - 1;

          for (size_t i = 0; i < row_end; ++ i)
          {
            if(bitmask(i) == in_lower)
            {
              vector_iterator const begin = l.partial_begin(i);
              vector_iterator const end = l.partial_end(i);
              for(vector_iterator e = begin;e!=end;++e)
                J(i,e.index()) = - *e;
              J(i,i) = value_traits::one();
            }
            if(bitmask(i) == in_upper)
            {
              vector_iterator const begin = u.partial_begin(i);
              vector_iterator const end = u.partial_end(i);
              for(vector_iterator e = begin;e!=end;++e)
                J(i,e.index()) = - *e;
              J(i,i) = value_traits::one();
            }
            if(bitmask(i) == in_active)
            {
              // J_i* = A_i*
              size_t const begin = A.index1_data()[i];
              size_t const end   = A.index1_data()[i+1];
              for (size_t k = begin; k < end; ++ k)
              {
                size_t const j = A.index2_data()[k];
                J(i,j) = A.value_data()[k];
              }
            }
          }
        }

      } // namespace detail
    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_JACOBIAN_H
#endif
