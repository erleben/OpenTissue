#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_PARTITIONED_JACOBIAN_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_PARTITIONED_JACOBIAN_H
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
        * Compute Partitioned Jacobian.
        *
        * @param A             The coefficient matrix of the linear equation y = A x + b
        * @param l             A functor used to retrieve information about the lower bound function.
        * @param u             A functor used to retrieve information about the upper bound function.
        * @param bitmask       A bitmask. 
        *                      The i'th value is equal to ``in active'' (=4) if and only if  y(i) \leq (x(i) - lo(i)) &&  y(i) \geq (x(i) - hi(i))
        *                      The i'th value is equal to ``in upper'' (=2) if and only if  y(i) < (x(i) - hi(i))
        *                      The i'th value is equal to ``in lower'' (=1) if and only if  y(i) > (x(i) - lo(i))
        * @param old2new       This vector holds an index permutation of old indices
        *                      to new indices. The permutation is such that the first sub-block of
        *                      the new vector corresponds to active constraints. The second corresponds
        *                      to lower constraints and the last sub-block corresponds to the upper
        *                      constraints set.
        * @param cnt_active    This argument holds the total number of variables in the set of active constraints.
        * @param cnt_inactive  This argument holds the total number of variables in the union of the set of lower and the set of upper constraints.
        * @param A_aa          Upon return this argument holds the sub-block of reordered jacobian corresponding to the row and column sets (active,active).
        * @param A_ab          Upon return this argument holds the sub-block of reordered jacobian corresponding to the row and column sets (active,inactive).
        * @param C             Upon return this argument holds the sub-block of reordered jacobian corresponding to the row and column sets (inactive,active).
        * @param D             Upon return this argument holds the sub-block of reordered jacobian corresponding to the row and column sets (inactive,inactive).
        */
        template<typename T, typename bound_function_type>
        inline void  compute_partitioned_jacobian( 
          ublas::compressed_matrix<T> const & A
          , bound_function_type const & l
          , bound_function_type const & u
          , ublas::vector<size_t> const & bitmask
          , ublas::vector<size_t> const & old2new
          , size_t const & cnt_active
          , size_t const & cnt_inactive
          , ublas::compressed_matrix<T> & A_aa
          , ublas::compressed_matrix<T> & A_ab
          , ublas::compressed_matrix<T> & C
          , ublas::compressed_matrix<T> & D
          )
        {
          using std::min;

          typedef typename bound_function_type::vector_iterator  vector_iterator;
          typedef typename OpenTissue::math::ValueTraits<T>      value_traits;

          static size_t const in_lower  = 1;
          static size_t const in_upper  = 2;
          static size_t const in_active = 4;

          A_aa.resize( cnt_active, cnt_active, false );
          A_ab.resize( cnt_active, cnt_inactive, false );
          C.resize( cnt_inactive, cnt_active, false );
          D.resize( cnt_inactive, cnt_inactive, false );

          size_t const row_end = A.filled1() - 1;
          for (size_t i_old = 0; i_old < row_end; ++i_old)
          {
            if( bitmask(i_old) == in_active )
            {
              size_t const i_new    = old2new( i_old );
              size_t const begin    = A.index1_data()[i_old];
              size_t const end      = A.index1_data()[i_old + 1];

              for (size_t j = begin; j < end; ++ j)
              {
                size_t const j_old = A.index2_data()[j];
                size_t const j_new = old2new( j_old );
                if( bitmask(j_old) == in_active )
                  A_aa( i_new, j_new            ) = A.value_data()[j];
                else
                  A_ab( i_new, j_new-cnt_active ) = A.value_data()[j];
              }
            }
          }

          size_t const n = cnt_active + cnt_inactive;

          for(size_t i_old = 0; i_old < n; ++i_old)
          {
            size_t const i_new = old2new( i_old );
            if( bitmask(i_old) == in_lower )
            {
              vector_iterator const begin = l.partial_begin( i_old );
              vector_iterator const end   = l.partial_end(   i_old );
              for(vector_iterator e = begin;e!=end;++e)
              {
                size_t const j_old = e.index();
                size_t const j_new = old2new( j_old );
                if( bitmask(j_old) == in_active )
                  C(i_new - cnt_active, j_new) = - *e;
                else
                  D(i_new - cnt_active ,j_new - cnt_active) = - *e;
              }
              D(i_new - cnt_active, i_new - cnt_active) = value_traits::one();
            }

            if( bitmask(i_old) == in_upper )
            {
              vector_iterator const begin = u.partial_begin( i_old );
              vector_iterator const end   = u.partial_end(   i_old );
              for(vector_iterator e = begin;e!=end;++e)
              {
                size_t const j_old = e.index();
                size_t const j_new = old2new( j_old );
                if( bitmask(j_old) == in_active )
                  C(i_new - cnt_active, j_new) = - *e;
                else
                  D(i_new - cnt_active ,j_new - cnt_active) = - *e;
              }
              D(i_new - cnt_active, i_new - cnt_active) = value_traits::one();
            }
          }
        }


      } // namespace detail
    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_BIG_COMPUTE_PARTITIONED_JACOBIAN_H
#endif
