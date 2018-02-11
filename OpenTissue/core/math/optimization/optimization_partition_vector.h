#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_PARTITION_VECTOR_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_PARTITION_VECTOR_H
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

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {
      /**
      * Partition vector.
      *
      * @param x             The value of the vector.
      * @param bitmask       A bitmask, the i'th element is equal to ``active'' value if and only if  y(i) \leq (x(i) - lo(i)) &&  y(i) \geq (x(i) - hi(i))
      * @param old2new       This vector holds an index permutation of old indices
      *                      to new indices. The permutation is such that the first sub-block of
      *                      the new vector corresponds to active constraints. The second corresponds
      *                      to lower constraints and the last sub-block corresponds to the upper
      *                      constraints set.
      * @param cnt_active    This argument holds the total number of variables in the set of active constraints.
      * @param cnt_inactive  This argument holds the total number of variables in the union of the set of lower and the set of upper constraints.
      * @param x_a           Upon return this vector holds the values of the sub-block of rhs that corresponds to the active constraints.
      * @param x_b           Upon return this vector holds the values of the sub-block of rhs that corresponds to the inactive constraints.
      */
      template<typename T >
      inline void  partition_vector( 
        ublas::vector<T> const & x
        , ublas::vector<size_t> const & bitmask
        , ublas::vector<size_t> const & old2new
        , size_t const & cnt_active
        , size_t const & cnt_inactive
        , ublas::vector<T>  & x_a
        , ublas::vector<T>  & x_b
        )
      {
        x_a.resize( cnt_active );
        x_b.resize( cnt_inactive );

        size_t n = x.size();

        for(size_t i_old = 0; i_old < n; ++i_old)
        {
          size_t i_new = old2new( i_old );
          if( bitmask(i_old) == IN_ACTIVE )
            x_a( i_new ) = x( i_old);
          else
            x_b( i_new-cnt_active ) = x( i_old);
        }
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_PARTITION_VECTOR_H
#endif
